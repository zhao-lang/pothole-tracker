//
//  UKF.swift
//  pothole tracker
//
//  Created by Zhao Lang on 1/2/19.
//  Copyright © 2019 nommer. All rights reserved.
//

import Foundation
import CoreLocation
import MetalPerformanceShaders
import Accelerate
import QuartzCore

func timeIt(_ block: () -> Void) -> CFTimeInterval {
    let startTime = CACurrentMediaTime()
    block()
    return CACurrentMediaTime() - startTime
}

class UKF: NSObject, CLLocationManagerDelegate {
    private var ready_: Bool!
    private var is_initialized_: Bool!
    private var initLat_: Float!
    private var initLon_: Float!
    private var time_: Double!
    
    private var x_: [Float]!
    private var x_prev_: [Float]!
    private var P_: [[Float]]!
    private var Xsig_pred_: [[Float]]!
    private var std_a_: Float!
    private var std_yawdd_: Float!
    private var std_gpslat_: Float!
    private var std_gpslon_: Float!
    private var std_gpsv_: Float!
    private var std_gpsphi_: Float!
    
    private var prev_pct_: Float!
    private var n_x_: Int!
    private var n_aug_: Int!
    private var lambda_: Int!
    private var weights_: [Float]!
    private var xsig_pred_: [[Float]]!
    private var nis_gps_: Float!
    
    private var locationMgr: CLLocationManager!
    private var userLocation: CLLocation!

    override init() {
        super.init()
        
        ready_ = false
        
        locationMgr = CLLocationManager()
        locationMgr.delegate = self
        locationMgr.desiredAccuracy = kCLLocationAccuracyBest
        locationMgr.requestAlwaysAuthorization()
        locationMgr.startUpdatingLocation()
        
        std_a_ = 3
        std_yawdd_ = Float.pi/16
        std_gpslat_ = 10
        std_gpslon_ = 10
        std_gpsv_ = 3
        std_gpsphi_ = Float.pi/16
        
        prev_pct_ = 0.0
//        prev_pct_ = 0.8
        n_x_ = 5
        n_aug_ = 7
        lambda_ = 3 - n_aug_
        
        nis_gps_ = 0
        
        x_ = [Float](repeating: 0, count: n_x_)
        P_ = [[Float]](repeating: [Float](repeating: 0, count: n_x_), count: n_x_)
        Xsig_pred_ = [[Float]](repeating: [Float](repeating: 0, count: n_x_), count: 2*n_aug_+1)
        
        for i in 0..<n_x_ {
            for j in 0..<n_x_ {
                if i == j {
                    P_[i][j] = Float(1)
                }
            }
        }
        
        weights_ = createWeights(n_aug_: n_aug_, lambda_: lambda_)
        
        is_initialized_ = false
    }
    
    func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
        userLocation = locations[0] as CLLocation
//        std_gpslat_ = Float(userLocation.horizontalAccuracy)
//        std_gpslon_ = Float(userLocation.horizontalAccuracy)
        
        if ready_ {
//            DispatchQueue.global(qos: .background).async {
//                print("Running UKF in background queue")
//
                let elapsed = timeIt {
                    self.processMeasurement()
                }
                print("time taken to process : ", elapsed)
                
//                DispatchQueue.main.async {
//                    print("This is run on the main queue, after the previous code in outer block")
//                }
//            }
        }
    }
    
    func locationManager(_ manager: CLLocationManager, didFailWithError error: Error) {
        print("Error \(error)")
    }
    
    func run() {
        print("running UKF...")
        
        while !is_initialized_ {
            processMeasurement()
        }

        print("done initializing")
    }
    
    func processMeasurement() {
        guard userLocation != nil else {
            return
        }
        
        let loc = userLocation!
        
        if !is_initialized_ {
            initLat_ = Float(loc.coordinate.latitude)
            initLon_ = Float(loc.coordinate.longitude)
            
            let px = Float(0)
            let py = Float(0)
            
            var v, phi: Float
            if loc.speed < 0 {
                v = 0
            } else {
                v = Float(loc.speed)
            }
            if loc.course < 0 {
                phi = deg2rad(Float(0))
            } else {
                phi = deg2rad(Float(90-loc.course))
            }
            
            x_[0] = px
            x_[1] = py
            x_[2] = v
            x_[3] = phi
//            x_[3] = 0
            x_[4] = 0
            
            x_prev_ = x_
            
            time_ = loc.timestamp.timeIntervalSince1970
            
            is_initialized_ = true
            ready_ = true
            return
        }
        
        // Prediction Step
        ready_ = false
        
        let dt: Double! = loc.timestamp.timeIntervalSince1970 - time_
        prediction(dt: dt)
        updateGPS(loc: loc)
        
        updateXPrev(&x_prev_, x: x_, prev_pct: prev_pct_)
        time_ = loc.timestamp.timeIntervalSince1970
        
        print("x_: ", x_)
        print("P_: ", P_)
        print("nis_gps_: ", nis_gps_)
        
        ready_ = true
    }
    
    //  Predicts sigma points, the state, and the state covariance matrix.
    func prediction(dt: Double) {
        let Xsig_aug_ = genSigma()
        predSigma(Xsig_aug_: Xsig_aug_, dt: dt)
        
        x_ = predictState(Xsig_pred: Xsig_pred_, weights: weights_, n_x_: n_x_)
        P_ = predictP(x: x_, Xsig_pred: Xsig_pred_, weights: weights_)
    }
    
    // update state and covar matrix using gps measurements
    func updateGPS(loc: CLLocation!) {
        let n_z_ = 4
        var Zsig = [[Float]](repeating: [Float](repeating: 0, count: n_z_), count: 2*n_aug_+1)
        
        for i in 0..<2*n_aug_+1 {
            Zsig[i] = makeZSig(Xsig_pred: Xsig_pred_[i], x_prev: x_prev_, n_z: n_z_)
        }
        
        let z_pred = predictZ(Zsig, x_prev: x_prev_, weights: weights_)
//        print(z_pred)
        
        var S = calcS(z_pred: z_pred, Zsig: Zsig, weights: weights_)
        
        // TODO factor to noise augmentation function
        S[0][0] += std_gpslon_ * std_gpslon_
        S[1][1] += std_gpslat_ * std_gpslat_
        if n_z_ > 2 {
           S[2][2] += std_gpsv_ * std_gpsv_
        }
        if n_z_ > 3 {
            S[3][3] += std_gpsphi_ * std_gpsphi_
        }
        
        let Tc = calcTc(x: x_, Xsig_pred: Xsig_pred_, z_pred: z_pred, Zsig: Zsig, weights: weights_)
        let Si = matrixInversion(S)
        let KalmanGain = calcKalmanGain(Tc: Tc, Si: Si)
        
        var z = gps2xy(loc: loc, initLat: initLat_, initLon: initLon_)
        var v, phi: Float
        if loc.speed < 0 {
            v = x_prev_[2]
        } else {
            v = Float(loc.speed)
        }
        if loc.course < 0 {
            phi = x_prev_[3]
        } else {
            phi = deg2rad(Float(90-loc.course))
        }
        z += [v, phi]
//        print("z: ", z)
        
        var z_diff = calcDiff(z, z_pred)
        if n_z_ > 3 {
            fixYaw(&z_diff, 3)
        }
        
        updateState(&x_, K: KalmanGain, z_diff: z_diff)
        updateCoVar(&P_, K: KalmanGain, S: S)
        nis_gps_ = calcNIS(z_diff: z_diff, Si: Si)
    }
    
    //  Generate sigma points
    func genSigma() -> [[Float]] {
        let x_aug = genXAug(x_: x_, n_x_: n_x_, n_aug_: n_aug_)
        let P_aug = genPAug(P_: P_, n_x_: n_x_, n_aug_: n_aug_, std_a_: std_a_, std_yawdd_: std_yawdd_)
        let L = lltDecomp(mat: P_aug, n: n_aug_)
        
        let Xsig_aug_ = genXsigAug(x_aug: x_aug, L: L, n_aug_: n_aug_, lambda_: lambda_)
        return Xsig_aug_
    }
    
    // Predict sigma points
    func predSigma(Xsig_aug_: [[Float]], dt: Double) {
        for i in 0..<2*n_aug_+1 {
            let px = Xsig_aug_[i][0]
            let py = Xsig_aug_[i][1]
            let v = Xsig_aug_[i][2]
            let yaw = Xsig_aug_[i][3]
            let yawd = Xsig_aug_[i][4]
            let nu_a = Xsig_aug_[i][5]
            let nu_yawdd = Xsig_aug_[i][6]
            
            let (px_p, py_p) = predictXY(px: px, py: py, v: v, yaw: yaw, yawd: yawd, nu_a: nu_a, dt: dt)
            let v_p = predictV(v: v, nu_a: nu_a, dt: dt)
            let yaw_p = predictYaw(yaw: yaw, yawd: yawd, nu_yawdd: nu_yawdd, dt: dt)
            let yawd_p = predictYawd(yawd: yawd, nu_yawdd: nu_yawdd, dt: dt)
  
            Xsig_pred_[i][0] = px_p;
            Xsig_pred_[i][1] = py_p;
            Xsig_pred_[i][2] = v_p;
            Xsig_pred_[i][3] = yaw_p;
            Xsig_pred_[i][4] = yawd_p;
            
            fixYaw(&Xsig_pred_[i], 3)
            fixYaw(&Xsig_pred_[i], 4)
        }
    }
    
//    func xy2gps(x: Float, y: Float, loc: CLLocation) -> [Float] {
//        var latMid, m_per_deg_lat, m_per_deg_lon, lat, lon: Float
//
//        latMid = Float((initLat_ + loc.coordinate.latitude) / 2.0) * Float.pi / 180
//
//        m_per_deg_lat = 111132.92 - 559.82 * cos(2.0 * latMid) + 1.175 * cos(4.0 * latMid) - 0.0023 * cos(6.0 * latMid)
//        m_per_deg_lon = 111412.84 * cos(latMid) - 93.5 * cos(3.0 * latMid) + 0.118 * cos(5.0 * latMid)
//
//        lat = y / m_per_deg_lat + Float(loc.coordinate.latitude)
//        lon = x / m_per_deg_lon + Float(loc.coordinate.longitude)
//
//        return [lat, lon]
//    }
    
    func mpsLLT() {
        let dev: MTLDevice! = MTLCreateSystemDefaultDevice()
        guard dev != nil else {
            fatalError("Error: This device does not support Metal")
        }
        
        guard MPSSupportsMTLDevice(dev) else {
            fatalError("Error: This device does not support Metal Performance Shaders")
        }
        
        let cmdQueue = dev!.makeCommandQueue()
        let commandBuffer = cmdQueue!.makeCommandBuffer()
        
        let arr: [Float] = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        let bufferStatus = dev.makeBuffer(length: 1 * MemoryLayout<Int>.stride, options: [])
        let bufferSrc = dev.makeBuffer(bytes: arr, length: 3 * 3 * MemoryLayout<Float>.stride, options: [])
        let bufferL = dev.makeBuffer(length: 3 * 3 * MemoryLayout<Float>.stride, options: [])
        
        let descSrc = MPSMatrixDescriptor(rows: 3, columns: 3, rowBytes: 3 * MemoryLayout<Float>.stride, dataType: .float32)
        let descL = MPSMatrixDescriptor(rows: 3, columns: 3, rowBytes: 3 * MemoryLayout<Float>.stride, dataType: .float32)
        
        let src = MPSMatrix(buffer: bufferSrc!, descriptor: descSrc)
        let L = MPSMatrix(buffer: bufferL!, descriptor: descL)
        
        let llt = MPSMatrixDecompositionCholesky(device: dev!, lower: true, order: 3)
        llt.encode(commandBuffer: commandBuffer!, sourceMatrix: src, resultMatrix: L, status: bufferStatus)
        
        let rawPointer = L.data.contents()
        let count = L.rows * L.columns
        let typedPointer = rawPointer.bindMemory(to: Float.self, capacity: count)
        let bufferedPointer = UnsafeMutableBufferPointer(start: typedPointer, count: count)
        
        print(Array(bufferedPointer))
    }
}

func createWeights(n_aug_: Int, lambda_: Int) -> [Float] {
    let weight = 0.5 / Float(n_aug_+lambda_);
    var weights = [Float](repeating: weight, count: 2*n_aug_+1)
    weights[0] = Float(lambda_)/Float(lambda_+n_aug_);
    return weights
}

func genXAug(x_: [Float], n_x_: Int, n_aug_: Int) -> [Float] {
    var x_aug = [Float](repeating: 0, count: n_aug_)
    for i in 0..<n_x_ {
        x_aug[i] = x_[i]
    }
    return x_aug
}

func genPAug(P_: [[Float]], n_x_: Int, n_aug_: Int, std_a_: Float, std_yawdd_: Float) -> [[Float]] {
    var P_aug_ = [[Float]](repeating: [Float](repeating: 0, count: n_aug_), count: n_aug_)
    for i in 0..<n_x_ {
        for j in 0..<n_x_ {
            P_aug_[i][j] = P_[i][j]
        }
    }
    P_aug_[n_x_][n_x_] = std_a_ * std_a_
    P_aug_[n_x_+1][n_x_+1] = std_yawdd_ * std_yawdd_
    return P_aug_
}

func genXsigAug(x_aug: [Float], L: [[Float]], n_aug_: Int, lambda_: Int) -> [[Float]] {
    var Xsig_aug = [[Float]](repeating: [Float](repeating: 0, count: n_aug_), count: 2*n_aug_+1)
    Xsig_aug[0] = x_aug
    for i in 0..<n_aug_ {
        for j in 0..<n_aug_ {
            Xsig_aug[i+1][j] = x_aug[j] + Float(sqrt(Double(lambda_+n_aug_))) * L[j][i]
            Xsig_aug[i+1+n_aug_][j] = x_aug[j] - Float(sqrt(Double(lambda_+n_aug_))) * L[j][i]
        }
    }
    return Xsig_aug
}

func predictXY(px: Float, py: Float, v: Float, yaw: Float, yawd: Float, nu_a: Float, dt: Double) -> (Float, Float) {
    var px_p, py_p: Float
    
    if abs(yawd) > 0.0001 {
        px_p = px + v/yawd * ( sin(yaw + yawd*Float(dt)) - sin(yaw) )
        py_p = py + v/yawd * ( cos(yaw) - cos(yaw + yawd*Float(dt)) )
    } else {
        px_p = px + v * Float(dt) * cos(yaw)
        py_p = py + v * Float(dt) * sin(yaw)
    }
    
    px_p = px_p + 0.5*nu_a*Float(dt*dt) * cos(yaw)
    py_p = py_p + 0.5*nu_a*Float(dt*dt) * sin(yaw)
    
    return (px_p, py_p)
}

func predictV(v: Float, nu_a: Float, dt: Double) -> Float {
    return v + nu_a * Float(dt)
}

func predictYaw(yaw: Float, yawd: Float, nu_yawdd: Float, dt: Double) -> Float {
    return yaw + yawd * Float(dt) + 0.5 * nu_yawdd * Float(dt*dt)
}

func predictYawd(yawd: Float, nu_yawdd: Float, dt: Double) -> Float {
    return yawd + nu_yawdd * Float(dt)
}

func predictState(Xsig_pred: [[Float]], weights: [Float], n_x_: Int) -> [Float] {
    var state = [Float](repeating: 0, count: n_x_)
    for i in 0..<n_x_ {
        state[i] = zip(Xsig_pred.map({$0[i]}), weights).map({$0.0 * $0.1}).reduce(0, +)
    }
    fixYaw(&state, 3)
    fixYaw(&state, 4)
    return state
}

func predictP(x: [Float], Xsig_pred: [[Float]], weights: [Float]) -> [[Float]] {
    let n_x_ = x.count
    
    var P = [[Float]](repeating: [Float](repeating: 0, count: n_x_), count: n_x_)
    for k in 0..<Xsig_pred.count {
        var x_diff = calcDiff(Xsig_pred[k], x)
        fixYaw(&x_diff, 3)
        fixYaw(&x_diff, 4)
        
        let p = vecOuterProduct(x_diff, x_diff, weight: weights[k])
        incrementMat(&P, a: p)
    }
    return P
}

func makeZSig(Xsig_pred: [Float], x_prev: [Float], n_z: Int) -> [Float] {
    var Zsig = [Float](repeating: 0, count: n_z)
    
    let px = Xsig_pred[0]
    let py = Xsig_pred[1]
    
    Zsig[0] = px
    Zsig[1] = py
    
    if n_z > 2 {
        Zsig[2] = Xsig_pred[2]
    }
    
    if n_z > 3 {
        let dx = px - x_prev[0]
        let dy = py - x_prev[1]
        
        Zsig[3] = atan2f(dy, dx)
    }
    
    return Zsig
}

func predictZ(_ Zsig: [[Float]], x_prev: [Float], weights: [Float]) -> [Float] {
    let n_z = Zsig[0].count
    var z_pred = [Float](repeating: 0, count: n_z)
    for i in 0..<n_z {
        z_pred[i] = zip(Zsig.map({$0[i]}), weights).map({$0.0 * $0.1}).reduce(0, +)
    }
    
//    if n_z > 3 {
//        let px = z_pred[0]
//        let py = z_pred[1]
//
//        let dx = px - x_prev[0]
//        let dy = py - x_prev[1]
//
//        z_pred[3] = atan2f(dy, dx)
//    }
    
    return z_pred
}

func calcS(z_pred: [Float], Zsig: [[Float]], weights: [Float]) -> [[Float]] {
    let n_z_ = z_pred.count
    
    var S =  [[Float]](repeating: [Float](repeating: 0, count: n_z_), count: n_z_)
    for k in 0..<Zsig.count {
        var z_diff = calcDiff(Zsig[k], z_pred)
        if n_z_ > 3 {
            fixYaw(&z_diff, 3)
        }
        
        let s = vecOuterProduct(z_diff, z_diff, weight: weights[k])
        incrementMat(&S, a: s)
    }
    return S
}

func calcTc(x: [Float], Xsig_pred: [[Float]], z_pred: [Float], Zsig: [[Float]], weights: [Float]) -> [[Float]] {
    let n_x_ = x.count
    let n_z_ = z_pred.count
    
    var Tc = [[Float]](repeating: [Float](repeating: 0, count: n_z_), count: n_x_)
    
    for k in 0..<Xsig_pred.count {
        var z_diff = calcDiff(Zsig[k], z_pred)
        if n_z_ > 3 {
            fixYaw(&z_diff, 3)
        }
        var x_diff = calcDiff(Xsig_pred[k], x)
        fixYaw(&x_diff, 3)
        fixYaw(&x_diff, 4)
        
        let t = vecOuterProduct(x_diff, z_diff, weight: weights[k])
        incrementMat(&Tc, a: t)
    }
    return Tc
}

func calcKalmanGain(Tc: [[Float]], Si: [[Float]]) -> [[Float]] {
    let n_x_ = Tc.count
    let n_z_ = Si.count
    
    var K_arr = [Float](repeating: 0, count: n_x_*n_z_)
    let Tc_arr: [Float] = flattenMatrix(Tc)
    let Si_arr: [Float] = flattenMatrix(Si)
    
    cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, Int32(n_x_), Int32(n_z_), Int32(n_z_), 1.0, Tc_arr, Int32(n_z_), Si_arr, Int32(n_z_), 0.0, &K_arr, Int32(n_z_))
    
    var K = [[Float]](repeating: [Float](repeating: 0, count: n_z_), count: n_x_)
    for i in 0..<n_x_ {
        for j in 0..<n_z_ {
            K[i][j] = K_arr[i*n_z_+j]
        }
    }
    
    return K
}

func updateState(_ x: inout [Float], K: [[Float]], z_diff: [Float]) {
    let n_x_ = x.count
    let n_z_ = K[0].count
    
    var kzd = [Float](repeating: 0, count: n_x_)
    cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, Int32(n_x_), 1, Int32(n_z_), 1.0, flattenMatrix(K), Int32(n_z_), z_diff, 1, 0.0, &kzd, 1)
    
    x = zip(x, kzd).map({$0.0 + $0.1})
}

func updateCoVar(_ P: inout [[Float]], K: [[Float]], S: [[Float]]) {
    let n_x_ = K.count
    let n_z_ = S.count
    
    var KS_arr = [Float](repeating: 0, count: n_x_*n_z_)
    cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, Int32(n_x_), Int32(n_z_), Int32(n_z_), 1.0, flattenMatrix(K), Int32(n_z_), flattenMatrix(S), Int32(n_z_), 0.0, &KS_arr, Int32(n_z_))
    
    var KSKt_arr = [Float](repeating: 0, count: n_x_*n_x_)
    cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasTrans, Int32(n_x_), Int32(n_x_), Int32(n_z_), 1.0, KS_arr, Int32(n_z_), flattenMatrix(K), Int32(n_z_), 0.0, &KSKt_arr, Int32(n_x_))
    
    var KSKt = [[Float]](repeating: [Float](repeating: 0, count: n_x_), count: n_x_)
    for i in 0..<n_x_ {
        for j in 0..<n_x_ {
            KSKt[i][j] += KSKt_arr[i*n_x_+j]
        }
    }
    
//    print("P pre update: ", P)
//    print("K: ", K)
//    print("S: ", S)
    
    for i in 0..<n_x_ {
        for j in 0..<n_x_ {
            P[i][j] -= KSKt[i][j]
        }
    }
}

func calcNIS(z_diff: [Float], Si: [[Float]]) -> Float {
    let n_z_ = z_diff.count
    
    var ZDtSi_arr = [Float](repeating: 0, count: n_z_)
    cblas_sgemm(CblasRowMajor, CblasTrans, CblasNoTrans, 1, Int32(n_z_), Int32(n_z_), 1.0, z_diff, 1, flattenMatrix(Si), Int32(n_z_), 0.0, &ZDtSi_arr, Int32(n_z_))
    
    var ZDtSiZD_arr = [Float](repeating: 0, count: 1)
    cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 1, 1, Int32(n_z_), 1.0, ZDtSi_arr, Int32(n_z_), z_diff, 1, 0.0, &ZDtSiZD_arr, 1)
    
    return ZDtSiZD_arr[0]
}

func updateXPrev(_ x_prev: inout [Float], x: [Float], prev_pct: Float) {
    for i in 0..<x_prev.count {
        x_prev[i] = prev_pct * x_prev[i] + (1-prev_pct) * x[i]
    }
}

func incrementMat(_ Mat: inout [[Float]], a: [[Float]]) {
    for i in 0..<Mat.count {
        for j in 0..<Mat[0].count {
            Mat[i][j] += a[i][j]
        }
    }
}

func fixYaw(_ x: inout [Float], _ idxYaw: Int) {
    while x[idxYaw] > Float.pi {
        x[idxYaw] -= Float(2) * Float.pi
    }
    while x[idxYaw] < -Float.pi {
        x[idxYaw] += Float(2) * Float.pi
    }
}

func calcDiff(_ x1: [Float], _ x2: [Float]) -> [Float] {
    return zip(x1, x2).map({$0.0 - $0.1})
}

func vecOuterProduct(_ vec1: [Float], _ vec2: [Float], weight: Float) -> [[Float]] {
    let n = vec1.count
    let m = vec2.count
    var p_arr = [Float](repeating: 0, count: n*m)
    cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, Int32(n), Int32(m), 1, weight, vec1, 1, vec2, Int32(m), 0.0, &p_arr, Int32(m))
    
    var p = [[Float]](repeating: [Float](repeating: 0, count: m), count: n)
    for i in 0..<n {
        for j in 0..<m {
            p[i][j] = p_arr[i*m+j]
        }
    }
    
    return p
}

func lltDecomp(mat: [[Float]], n: Int) -> [[Float]] {
    var A: [Float] = flattenMatrix(mat)
    let uplo = UnsafeMutablePointer<Int8>(mutating: ("L" as NSString).utf8String)
    var N = Int32(n)
    var lda = Int32(n)
    var error = Int32(0)
    // Perform LLT decomposition
    spotrf_(uplo, &N, &A, &lda, &error)
    var res = [[Float]](repeating: [Float](repeating: 0, count: n), count: n)
    for i in 0..<n {
        for j in 0..<n {
            if j >= i {
                res[j][i] = A[i*n+j]
            }
        }
    }
    return res
}

func matrixInversion(_ Mat: [[Float]]) -> [[Float]] {
    let n = Mat.count
    var outMat = [[Float]](repeating: [Float](repeating: 0, count: n), count: n)
    var inv_arr: [Float] = flattenMatrix(Mat)
    var N = Int32(n)
    var M = Int32(n)
    var lda = Int32(n)
    var lwork = Int32(n)
    var pivots = [Int32](repeating: 0, count: n)
    var workspace = [Float](repeating: 0, count: n)
    var error = Int32(0)
    // Perform LU factorization
    sgetrf_(&M, &N, &inv_arr, &lda, &pivots, &error)
    // Calculate inverse from LU factorization
    sgetri_(&N, &inv_arr, &lda, &pivots, &workspace, &lwork, &error)
    
    for i in 0..<n {
        for j in 0..<n {
            outMat[i][j] = inv_arr[i*n+j]
        }
    }
    
    return outMat
}

func flattenMatrix<T>(_ mat: [Array<T>]) -> Array<T> {
    return mat.flatMap({$0})
}

func gps2xy(loc: CLLocation, initLat: Float, initLon: Float) -> [Float] {
    var latMid, m_per_deg_lat, m_per_deg_lon, dx, dy: Float
    
    latMid = ((initLat + Float(loc.coordinate.latitude)) / 2.0) * Float.pi / 180
    
    m_per_deg_lat = 111132.92 - 559.82 * cos(2.0 * latMid) + 1.175 * cos(4.0 * latMid) - 0.0023 * cos(6.0 * latMid)
    m_per_deg_lon = 111412.84 * cos(latMid) - 93.5 * cos(3.0 * latMid) + 0.118 * cos(5.0 * latMid)
    
    dy = (Float(loc.coordinate.latitude) - initLat) * m_per_deg_lat
    dx = (Float(loc.coordinate.longitude) - initLon) * m_per_deg_lon
    
    return [dx, dy]
}

func deg2rad(_ deg: Float) -> Float {
    return deg * Float.pi / 180
}

func rad2deg(_ rad: Float) -> Float {
    return rad * 180 / Float.pi
}
