//
//  pothole_trackerTests.swift
//  pothole trackerTests
//
//  Created by Zhao Lang on 1/2/19.
//  Copyright © 2019 nommer. All rights reserved.
//

import XCTest
import CoreLocation
@testable import pothole_tracker

class pothole_trackerTests: XCTestCase {

    override func setUp() {
        // Put setup code here. This method is called before the invocation of each test method in the class.
    }

    override func tearDown() {
        // Put teardown code here. This method is called after the invocation of each test method in the class.
    }
    
    func testFlattenMatrix() {
        let src: [[Int]] = [[1,2,3],[4,5,6],[7,8,9]]
        let res = flattenMatrix(src)
        let exp: [Int] = [1,2,3,4,5,6,7,8,9]
        XCTAssertEqual(res, exp)
    }
    
    func testGps2xy() {
        let loc = CLLocation(latitude: 40, longitude: -79)
        let xy = gps2xy(loc: loc, initLat: 41, initLon: -80)
        XCTAssert(xy[0] > 0)
        XCTAssert(xy[1] < 0)
        XCTAssert(round(sqrt(xy[0]/1000*xy[0]/1000+xy[1]/1000*xy[1]/1000)*100)/100 == 139.70)
    }
    
    func testCreateWeights() {
        let n_aug_ = 7
        let lambda_ = 3 - n_aug_
        let weights = createWeights(n_aug_: n_aug_, lambda_: lambda_)
        
        let sum = weights.reduce(0, +)
        
        var exp = [Float](repeating: 1/6, count: 15)
        exp[0] = -4/3
        
        XCTAssertEqual(sum, 1.0)
        XCTAssertEqual(weights, exp)
    }
    
    func testGenXAug() {
        let x_: [Float] = [1,2,3,4,5]
        let n_x_ = 5
        let n_aug_ = 7
        let x_aug = genXAug(x_: x_, n_x_: n_x_, n_aug_: n_aug_)
        let exp: [Float] = [1,2,3,4,5,0,0]
        XCTAssertEqual(x_aug, exp)
    }
    
    func testGenPAug() {
        let P_: [[Float]] = [[1,2,3],[4,5,6],[7,8,9]]
        let n_x_ = 3
        let n_aug_ = 5
        let std_a_: Float = 3
        let std_yawdd_: Float = 0.5
        let P_aug_ = genPAug(P_: P_, n_x_: n_x_, n_aug_: n_aug_, std_a_: std_a_, std_yawdd_: std_yawdd_)
        let exp: [[Float]] = [[1,2,3,0,0],[4,5,6,0,0],[7,8,9,0,0],[0,0,0,9,0],[0,0,0,0,0.25]]
        XCTAssertEqual(P_aug_, exp)
    }
    
    func testGenXsigAug() {
        let x_aug: [Float] = [1,2,3,4,5]
        let L: [[Float]] = [[1,2,3,4,5],[6,7,8,9,10],[11,12,13,14,15],[16,17,18,19,20],[21,22,23,24,25]]
        let n_aug_ = 5
        let lambda_ = 4 - n_aug_
        let Xsig_aug = genXsigAug(x_aug: x_aug, L: L, n_aug_: n_aug_, lambda_: lambda_)
        
        var exp = [[Float]](repeating: [Float](repeating: 0, count: n_aug_), count: 2*n_aug_+1)
        exp[0] = [1,2,3,4,5]
        exp[1] = [1+2*L[0][0], 2+2*L[1][0], 3+2*L[2][0], 4+2*L[3][0], 5+2*L[4][0]]
        exp[2] = [1+2*L[0][1], 2+2*L[1][1], 3+2*L[2][1], 4+2*L[3][1], 5+2*L[4][1]]
        exp[3] = [1+2*L[0][2], 2+2*L[1][2], 3+2*L[2][2], 4+2*L[3][2], 5+2*L[4][2]]
        exp[4] = [1+2*L[0][3], 2+2*L[1][3], 3+2*L[2][3], 4+2*L[3][3], 5+2*L[4][3]]
        exp[5] = [1+2*L[0][4], 2+2*L[1][4], 3+2*L[2][4], 4+2*L[3][4], 5+2*L[4][4]]
        exp[6] = [1-2*L[0][0], 2-2*L[1][0], 3-2*L[2][0], 4-2*L[3][0], 5-2*L[4][0]]
        exp[7] = [1-2*L[0][1], 2-2*L[1][1], 3-2*L[2][1], 4-2*L[3][1], 5-2*L[4][1]]
        exp[8] = [1-2*L[0][2], 2-2*L[1][2], 3-2*L[2][2], 4-2*L[3][2], 5-2*L[4][2]]
        exp[9] = [1-2*L[0][3], 2-2*L[1][3], 3-2*L[2][3], 4-2*L[3][3], 5-2*L[4][3]]
        exp[10] = [1-2*L[0][4], 2-2*L[1][4], 3-2*L[2][4], 4-2*L[3][4], 5-2*L[4][4]]
        
        XCTAssertEqual(Xsig_aug, exp)
    }
    
    func testPredictXY() {
        let px: Float = 1.0
        let py: Float = 2.0
        let v: Float = 3.0
        let yaw = deg2rad(60)
        let yawd = deg2rad(30)
        let nu_a: Float = 4.0
        let dt = 0.5
        
        let (px_p, py_p) = predictXY(px: px, py: py, v: v, yaw: yaw, yawd: yawd, nu_a: nu_a, dt: dt)
        
        var exp_x1: Float = 1.0 + 3.0/(30*Float.pi/180)*(sin((60+30*0.5)*Float.pi/180) - sin(60*Float.pi/180))
        exp_x1 = exp_x1 + 0.5*4.0*0.5*0.5*cos(60*Float.pi/180)
        let exp_x2: Float = 1.822387
        
        var exp_y1: Float = 2.0 + 3.0/(30*Float.pi/180)*(cos(60*Float.pi/180) - cos((60+30*0.5)*Float.pi/180))
        exp_y1 = exp_y1 + 0.5*4.0*0.5*0.5*sin(60*Float.pi/180)
        let exp_y2: Float = 3.814878
        
        XCTAssertEqual(px_p, exp_x1)
        XCTAssertEqual(round(exp_x1*10000), round(exp_x2*10000))
        XCTAssertEqual(py_p, exp_y1)
        XCTAssertEqual(round(exp_y1*10000), round(exp_y2*10000))
    }
    
    func testPredictXYZeroYawd() {
        let px: Float = 1.0
        let py: Float = 2.0
        let v: Float = 3.0
        let yaw = deg2rad(60)
        let yawd: Float = 0
        let nu_a: Float = 4.0
        let dt = 0.5
        
        let (px_p, py_p) = predictXY(px: px, py: py, v: v, yaw: yaw, yawd: yawd, nu_a: nu_a, dt: dt)
        
        var exp_x1: Float = 1.0 + 3.0*0.5*cos(60*Float.pi/180)
        exp_x1 = exp_x1 + 0.5*4.0*0.5*0.5*cos(60*Float.pi/180)
        let exp_x2: Float = 2
        
        var exp_y1: Float = 2.0 + 3.0*0.5*sin(60*Float.pi/180)
        exp_y1 = exp_y1 + 0.5*4.0*0.5*0.5*sin(60*Float.pi/180)
        let exp_y2: Float = 3.732051
        
        XCTAssertEqual(px_p, exp_x1)
        XCTAssertEqual(round(exp_x1*10000), round(exp_x2*10000))
        XCTAssertEqual(py_p, exp_y1)
        XCTAssertEqual(round(exp_y1*10000), round(exp_y2*10000))
    }
    
    func testPredictV() {
        let v: Float = 10
        let nu_a: Float = 5
        let dt = 0.5
        let v_p = predictV(v: v, nu_a: nu_a, dt: dt)
        
        let exp: Float = 10+5*0.5
        
        XCTAssertEqual(v_p, exp)
    }
    
    func testPredictYaw() {
        let yaw = deg2rad(90)
        let yawd = deg2rad(45)
        let nu_yawdd = deg2rad(30)
        let dt = 0.5
        let yawp = predictYaw(yaw: yaw, yawd: yawd, nu_yawdd: nu_yawdd, dt: dt)
        
        let exp = deg2rad(Float(90 + 45*0.5 + 0.5*30*0.5*0.5))
        
        XCTAssertEqual(round(yawp*10000), round(exp*10000))
    }
    
    func testPredictYawd() {
        let yawd = deg2rad(45)
        let nu_yawdd = deg2rad(30)
        let dt = 0.5
        let yawd_p = predictYawd(yawd: yawd, nu_yawdd: nu_yawdd, dt: dt)
        
        let exp = deg2rad(60)
        
        XCTAssertEqual(yawd_p, exp)
    }
    
    func testPredictState() {
        let xsig_pred: [[Float]] = [
            [1,2,3,4,5],
            [6,7,8,9,10],
            [11,12,13,14,15]
        ]
        let weights: [Float] = [1/3, 1/3, 1/3]
        let n_x_ = 4
        let state = predictState(Xsig_pred: xsig_pred, weights: weights, n_x_: n_x_)
        
        let exp: [Float] = [18/3, 21/3, 24/3, 27/3]
        
        XCTAssertEqual(state, exp)
    }
    
    func testCalcKalmanGain() {
        let Tc: [[Float]] = [
            [1,2,3],
            [4,5,6],
            [7,8,9],
            [10,11,12]
        ]
        let Si: [[Float]] = [
            [0.1,0.2,0.3],
            [0.4,0.5,0.6],
            [0.7,0.8,0.9]
        ]
        var res = calcKalmanGain(Tc: Tc, Si: Si)
        for i in 0..<res.count {
            for j in 0..<res[0].count {
                res[i][j] = round(res[i][j]*100000)/100000
            }
        }
        
        let exp: [[Float]] = [
            [3,3.6,4.2],
            [6.6,8.1,9.6],
            [10.2,12.6,15],
            [13.8,17.1,20.4]
        ]
        
        XCTAssertEqual(res, exp)
    }
    
    func testUpdateState() {
        var x: [Float] = [1,2,3,4]
        let z_diff: [Float] = [0.1,0.2,0.3]
        let K: [[Float]] = [
            [1,2,3],
            [4,5,6],
            [7,8,9],
            [10,11,12]
        ]
        updateState(&x, K: K, z_diff: z_diff)
        
        let exp: [Float] = [2.4,5.2,8,10.8]
        
        XCTAssertEqual(x, exp)
    }
    
    func testUpdateCoVar() {
        var P: [[Float]] = [
            [1,2,3,4],
            [5,6,7,8],
            [9,10,11,12],
            [13,14,15,16]
        ]
        let K: [[Float]] = [
            [1.1,1.2,1.3],
            [1.4,1.5,1.6],
            [1.7,1.8,1.9],
            [2.0,2.1,2.2]
        ]
        let S: [[Float]] = [
            [0.1,0.2,0.3],
            [0.4,0.5,0.6],
            [0.7,0.8,0.9]
        ]
        updateCoVar(&P, K: K, S: S)
        for i in 0..<P.count {
            for j in 0..<P[0].count {
                P[i][j] = round(P[i][j]*100000)/100000
            }
        }
        
        let exp: [[Float]] = [
            [-5.768, -6.442, -7.116, -7.79],
            [-3.406, -4.485, -5.564, -6.643],
            [-1.044, -2.528, -4.012, -5.496],
            [1.318, -0.571, -2.46, -4.349]
        ]
        
        XCTAssertEqual(P, exp)
    }
    
    func testCalcNIS() {
        let z_diff: [Float] = [0.1,0.2,0.3]
        let Si: [[Float]] = [
            [1,2,3],
            [4,5,6],
            [7,8,9]
        ]
        let nis = calcNIS(z_diff: z_diff, Si: Si)
        
        let exp: Float = 2.28
        
        XCTAssertEqual(nis, exp)
    }
    
    func testUpdateXPrev() {
        var x_prev: [Float] = [1,2,3,4,5]
        let x: [Float] = [6,7,8,9,10]
        let prev_pct: Float = 0.8
        updateXPrev(&x_prev, x: x, prev_pct: prev_pct)
        
        let exp: [Float] = [0.8+1.2, 1.6+1.4, 2.4+1.6, 3.2+1.8, 4.0+2.0]
        
        XCTAssertEqual(x_prev, exp)
    }
    
    func testIncrementMat() {
        var P: [[Float]] = [
            [1,2,3],
            [4,5,6],
            [7,8,9],
            [10,11,12]
        ]
        
        let p: [[Float]] = [
            [0.1,0.2,0.3],
            [0.4,0.5,0.6],
            [0.7,0.8,0.9],
            [1.0,1.1,1.2]
        ]
        
        incrementMat(&P, a: p)
        
        let exp: [[Float]] = [
            [1.1,2.2,3.3],
            [4.4,5.5,6.6],
            [7.7,8.8,9.9],
            [11.0,12.1,13.2]
        ]
        
        XCTAssertEqual(P, exp)
    }
    
    func testFixYaw() {
        var x1: [Float] = [1, 2, 4.1*Float.pi]
        fixYaw(&x1, 2)
        x1[2] = round(x1[2]*100000)/100000
        let exp1: [Float] = [1, 2, round(0.1*Float.pi*100000)/100000]
        
        var x2: [Float] = [1, 2, 3.1*Float.pi]
        fixYaw(&x2, 2)
        x2[2] = round(x2[2]*100000)/100000
        let exp2: [Float] = [1, 2, round(-0.9*Float.pi*100000)/100000]
        
        var x3: [Float] = [1, 2, -4.1*Float.pi]
        fixYaw(&x3, 2)
        x3[2] = round(x3[2]*100000)/100000
        let exp3: [Float] = [1, 2, round(-0.1*Float.pi*100000)/100000]
        
        var x4: [Float] = [1, 2, -3.1*Float.pi]
        fixYaw(&x4, 2)
        x4[2] = round(x4[2]*100000)/100000
        let exp4: [Float] = [1, 2, round(0.9*Float.pi*100000)/100000]
        
        XCTAssertEqual(x1, exp1)
        XCTAssertEqual(x2, exp2)
        XCTAssertEqual(x3, exp3)
        XCTAssertEqual(x4, exp4)
    }
    
    func testMakeZSig() {
        let xsig_pred: [Float] = [1,2,3,4,5]
        let x_prev: [Float] = [-1, -2]
        let zsig = makeZSig(Xsig_pred: xsig_pred, x_prev: x_prev, n_z: 4)
        
        let angle = atan2f(4, 2)
        let exp_angle: Float = 1.107149
        XCTAssertEqual(round(angle*1000000)/1000000, exp_angle)
        
        let exp: [Float] = [1,2,3,atan2f(4,2)]
        XCTAssertEqual(zsig, exp)
    }
    
    func testPredictZ() {
        let zsig: [[Float]] = [
            [1,2,3,4],
            [5,6,7,8],
            [9,10,11,12]
        ]
        let weights: [Float] = [1/4,1/4,1/4,1/4]
        let z_pred = predictZ(zsig, x_prev: [0, 0], weights: weights)
        
        let exp: [Float] = [15/4,18/4,21/4,24/4]
        
        XCTAssertEqual(z_pred, exp)
    }
    
    func testCalcDiff() {
        let x1: [Float] = [5, 7, 9]
        let x2: [Float] = [1, 2, 3]
        let diff = calcDiff(x1, x2)
        
        let exp: [Float] = [4, 5, 6]
        
        XCTAssertEqual(diff, exp)
    }
    
    func testVecByVecT() {
        let vec: [Float] = [1,2,3,4,5]
        let weight: Float = 2
        let res = vecOuterProduct(vec, vec, weight: weight)
        let exp: [[Float]] = [
            [2,4,6,8,10],
            [4,8,12,16,20],
            [6,12,18,24,30],
            [8,16,24,32,40],
            [10,20,30,40,50],
        ]
        
        XCTAssertEqual(res, exp)
        
        let vec2: [Float] = [6,7,8]
        let res2 = vecOuterProduct(vec, vec2, weight: weight)
        let exp2: [[Float]] = [
            [12,14,16],
            [24,28,32],
            [36,42,48],
            [48,56,64],
            [60,70,80],
        ]
        
        XCTAssertEqual(res2, exp2)
        
        let res3 = vecOuterProduct(vec2, vec, weight: weight)
        let exp3: [[Float]] = [
            [12,24,36,48,60],
            [14,28,42,56,70],
            [16,32,48,64,80],
        ]
        
        XCTAssertEqual(res3, exp3)
    }
    
    func testLLtDecomp() {
        let mat: [[Float]] = [[4,12,-16],[12,37,-43],[-16,-43,98]]
        let res = lltDecomp(mat: mat, n: 3)
        let exp: [[Float]] = [[2,0,0],[6,1,0],[-8,5,3]]
        XCTAssertEqual(res, exp)
    }
    
    func testMatrixInversion() {
        let mat: [[Float]] = [
            [1,2,3],
            [2,4,5],
            [3,5,6]
        ]
        var res = matrixInversion(mat)
        for i in 0..<res.count {
            for j in 0..<res[0].count {
                res[i][j] = round(res[i][j]*100000)/100000
            }
        }
        
        let exp: [[Float]] = [
            [1,-3,2],
            [-3,3,-1],
            [2,-1,0]
        ]
        
        XCTAssertEqual(res, exp)
    }
    
    func testDeg2rad() {
        let r0 = deg2rad(0)
        let r60 = deg2rad(60)
        let r90 = deg2rad(90)
        let r120 = deg2rad(120)
        let r180 = deg2rad(180)
        
        XCTAssertEqual(r0, 0)
        XCTAssertEqual(round(r60*10000)/10000, 1.0472)
        XCTAssertEqual(round(r90*10000)/10000, 1.5708)
        XCTAssertEqual(round(r120*10000)/10000, 2.0944)
        XCTAssertEqual(r180, Float.pi)
    }

    func testPerformanceExample() {
        // This is an example of a performance test case.
        self.measure {
            // Put the code you want to measure the time of here.
        }
    }

}
