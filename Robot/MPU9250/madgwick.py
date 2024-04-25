import numpy as np
from scipy.io import loadmat
from math import sin, cos, tan
import matplotlib.pyplot as plt
from navimath import *

# Dataset selection
# f_number
# 1: Example data provided by Magdwich
# 2: Real IMU data provided by Witmotion
# 3: IMU data provided by Understanding Kalman filter
# 4: Matlab UAV trajectory generated
f_number = 1

if f_number == 1:
    # Example Data
    ExData1 = loadmat('..\..\Data\ExampleData.mat')
    Gyroscope = np.deg2rad(ExData1['Gyroscope'])
    Accelerometer = ExData1['Accelerometer']
    Magnetometer = ExData1['Magnetometer']
    time = ExData1['time']
    Ts = time[1]-time[0]
    totalLen = Accelerometer.shape[0]
    SamplPeriod = Ts
    q0 = [1, 0, 0, 0]
elif f_number ==4:
    # Example Data
    # Example Data
    #ExData1 = loadmat('..\..\Data\logged_imu_data_No_sensor_bias.mat')
    ExData1 = loadmat('..\..\Data\logged_imu_data_Yes_sensor_bias.mat')
    Accelerometer_NED = ExData1['Accelerometer_NED']
    Accelerometer = ExData1['Accelerometer_IMU']
    Gyroscope_NED = ExData1['Gyroscope_NED']
    Gyroscope = ExData1['Gyroscope_IMU']
    Magnetometer = ExData1['Magnetometer_IMU']
    GPS_Pos_NED = ExData1['GPS_Position']
    GPS_Vel_NED = ExData1['GPS_Velocity']
    GPS_Hz = ExData1['GPS_Hz']
    Position_True = ExData1['Position_True']
    Position_Estimate = ExData1['Position_Estimate']
    Orient_Quat_True = ExData1['Orientation_True_Quaternion']
    Orient_Euler_True = ExData1['Orientation_True_Euler'] # yaw, pitch, roll
    Orient_Quat_Est = ExData1['Orientation_Estimate_Quaternion'] #Orientation Quaternion Estimation

    Velocity_True = ExData1['Velocity_True']
    Ts = 1/ExData1['sample_per_sec'][0,0]
    totalLen = Accelerometer.shape[0]
    SamplPeriod = Ts
    q0 = Orient_Quat_True[0,:]
    
# Madgwick filter package
# Quaternion production
# a: quaternion 1X4 row vector
# b: quaternion 1X4 row vector
# return: quaternion 1X4 row vector
def quaternProd(a, b):
    row = np.zeros((4))
    
    a1 = a[0]
    a2 = a[1]
    a3 = a[2]
    a4 = a[3]
    
    b1 = b[0]
    b2 = b[1]
    b3 = b[2]
    b4 = b[3]
    
    row[0] = a1*b1 - a2*b2 - a3*b3 - a4*b4
    row[1] = a1*b2 + a2*b1 + a3*b4 - a4*b3
    row[2] = a1*b3 - a2*b4 + a3*b1 + a4*b2
    row[3] = a1*b4 + a2*b3 - a3*b2 + a4*b1
    
    return row

# Quaternion conjugation
# q: quaternion 1X4 row vector
# return: conjugated quaternion
def quaternConj(q):
    q_conj = np.zeros((4))
    
    q_conj[0] = q[0]
    q_conj[1] = -q[1]
    q_conj[2] = -q[2]
    q_conj[3] = -q[3]
    
    return q_conj

# Gradient descent loss function
# SE_q: Normalized orientation quaternion ==> Earth frame with respec to Sensor frame
# E_d: Normalized reference frame measurement vector1X3 row vector
# S_s: Sensor frame measurement vector1X3 row vector
# return: 1X3 Column vector
def f(SE_q, E_d, S_s):
    # SE_q: quaternion 1X4
    # E_d: 1X4 reference 물리량 [0 dx dy dz]
    # S_s: 1X4 측정 물리량 [0 sx sy sz]
    row = np.zeros((3,1))
    
    # Reference frame measurement vector
    dx = E_d[0]
    dy = E_d[1]
    dz = E_d[2]
    
    # Sensor frame measurement vector
    sx = S_s[0]
    sy = S_s[1]
    sz = S_s[2]
    
    # Orientation quaternion vector
    q1 = SE_q[0]
    q2 = SE_q[1]
    q3 = SE_q[2]
    q4 = SE_q[3]
    
    row[0,0] = 2.0*dx*(0.5-q3**2-q4**2) + 2.0*dy*(q1*q4+q2*q3) + 2.0*dz*(q2*q4-q1*q3)-sx
    row[1,0] = 2.0*dx*(q2*q3-q1*q4) + 2.0*dy*(0.5-q2**2-q4**2) + 2.0*dz*(q1*q2+q3*q4)-sy
    row[2,0] = 2.0*dx*(q1*q3 + q2*q4) + 2.0*dy*(q3*q4- q1*q2) + 2.0*dz*(0.5-q2**2-q3**2)-sz
    
    return row

# Jacobian of loss function
# SE_q: Normalized orientation quaternion ==> Earth frame with respec to Sensor frame
# E_d: Normalized reference frame measurement vector1X3 row vector
# return: 3X4 Jacibian matrix
def J(SE_q, E_d):
    row = np.zeros((3,4))
    
    q1 = SE_q[0]
    q2 = SE_q[1]
    q3 = SE_q[2]
    q4 = SE_q[3]
    
    dx = E_d[0]
    dy = E_d[1]
    dz = E_d[2]
    
    row[0, 0] = 2.0*dy*q4 - 2.0*dz*q3
    row[0, 1] = 2.0*dy*q3 + 2.0*dz*q4
    row[0, 2] = -4.0*dx*q3 + 2.0*dy*q2-2.0*dz*q1
    row[0, 3] = -4.0*dx*q4+2.0*dy*q1 + 2.0*dz*q2
    
    row[1, 0] = -2.0*dx*q4+2.0*dz*q2
    row[1, 1] = 2.0*dx*q3-4.0*dy*q2+2.0*dz*q1
    row[1, 2] = 2.0*dx*q2+2.0*dz*q4
    row[1, 3] = -2.0*dx*q1-4.0*dy*q4+2.0*dz*q3
    
    row[2, 0] = 2.0*dx*q3-2.0*dy*q2
    row[2, 1] = 2.0*dx*q4-2.0*dy*q1-4.0*dz*q2
    row[2, 2] = 2.0*dx*q1+2.0*dy*q4-4.0*dz*q3
    row[2, 3] = 2.0*dx*q2+2.0*dy*q3
    
    return row

# Madgwick AHRS
# Orientation estimation using IMU data only
# Accelerometer: sensor frame acceleration measurement 1X3 row vector
# Gyroscope: sensor frame angular rate measurement 1X3 row vector
# q: quaternion 1X4 row vector
# beta: gradient descent scale 0~1
# SamplePeriod: sample period
# return: quaternion 1X4 row vector
def Madgwick_IMU(Accelerometer, Gyroscope, q, beta, SamplPeriod):
    Accel_norm = np.linalg.norm(Accelerometer)
    Accel_normalized = Accelerometer / Accel_norm
    
    g_n = np.array([0,0,1]) # NED 좌표계의 지구 중력 가속도
    E_d = g_n
    S_s = Accel_normalized
    
    F = f(q, E_d, S_s)
    Delta = J(q, E_d)
    
    # Measurement step calculation
    # step: column vector
    step = np.matmul((np.transpose(Delta)),F)
    step = step / np.linalg.norm(step)
    
    # Quaternion update
    # qDot: row vector
    qDot = 0.5 * quaternProd(q, np.array([0, Gyroscope[0], Gyroscope[1], Gyroscope[2]])) - beta * np.transpose(step)
    
    # Measurement update
    # q: row vector
    q = q+qDot*SamplPeriod
    
    # Normalization
    q = q/np.linalg.norm(q)
    q = np.reshape(q, (4))
    
    return q

# Madgwick AHRS
# Orientation estimation using IMU data + Magnetormeter
# Accelerometer: sensor frame acceleration measurement 1X3 row vector
# Gyroscope: sensor frame angular rate measurement 1X3 row vector
# Magnetometer: sensor frame magnetic flux measurement 1X3 row vector
# q: quaternion 1X4 row vector
# beta: gradient descent scale 0~1
# SamplePeriod: sample period
# return: quaternion 1X4 row vector
def Madgwick_IMU_Magnetometer(Accelerometer, Gyroscope, Magnetometer, q, beta, SamplPeriod):
    # Normalization Accelerometer Measurement
    Accel_norm = np.linalg.norm(Accelerometer)
    Accel_normalized = Accelerometer / Accel_norm
    g_n = np.array([0,0,1]) # Reference vector
    E_d = g_n
    S_s = Accel_normalized
    F_Accel = f(q, E_d, S_s)
    Delta_Accel = J(q, E_d)
    # Nomalization Magnetometer Measurement
    Magnet_norm = np.linalg.norm(Magnetometer)
    Magnet_normalized = Magnetometer / Magnet_norm
    # Reference direction of Earth;s magnetic field
    # Rotation from body to inertial frame using quaternion
    h = quaternProd(q, quaternProd([0,
    Magnet_normalized[0],Magnet_normalized[1], Magnet_normalized[2]],
    quaternConj(q)))
    b = [0, np.linalg.norm([h[1], h[2]]), 0, h[3]] # Reference vector
    E_d = b[1:4]
    S_s = Magnet_normalized
    F_Magnet = f(q, E_d, S_s)
    Delta_Magnet = J(q, E_d)
    F = np.vstack((F_Accel,F_Magnet))
    Delta = np.vstack((Delta_Accel,Delta_Magnet))
    # Measurement step calculation
    # step: column vector
    step = np.matmul((np.transpose(Delta)),F)
    step = step / np.linalg.norm(step)
    # Quaternion update
    # qDot: row vector
    qDot = 0.5 * quaternProd(q, np.array([0, Gyroscope[0], Gyroscope[1],
    Gyroscope[2]])) - beta * np.transpose(step)
    # Measurement update
    # q: row vector
    q = q+qDot*SamplPeriod
    # Normalization
    q = q/np.linalg.norm(q)
    q = np.reshape(q, (4))
    return q