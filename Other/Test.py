import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import least_squares, minimize_scalar

def model_signal(theta, A1, A2, theta_offset, offset):
    a = A1 * np.cos(theta) + A2 * np.cos(9*theta + theta_offset) + offset
    b = A1 * np.sin(theta) + A2 * np.sin(9*theta + theta_offset) + offset
    return a, b

def residuals(p, theta, a_meas, b_meas):
    A1, A2c, A2s, offset = p
    a_pred = A1 * np.cos(theta) + A2c * np.cos(9*theta) - A2s * np.sin(9*theta) + offset
    b_pred = A1 * np.sin(theta) + A2s * np.cos(9*theta) + A2c * np.sin(9*theta) + offset
    return np.concatenate([a_pred - a_meas, b_pred - b_meas])

def fit_parameters(theta, a_noisy, b_noisy):
    p0 = [1.0, 0.1, 0.1, 0.0]
    bounds = ([0, -np.inf, -np.inf, -np.inf], [np.inf, np.inf, np.inf, np.inf])
    result = least_squares(residuals, p0, args=(theta, a_noisy, b_noisy), bounds=bounds)
    A1, A2c, A2s, offset = result.x
    A2 = np.sqrt(A2c**2 + A2s**2)
    theta_offset = np.arctan2(A2s, A2c)
    return A1, A2, theta_offset, offset

def estimate_theta_from_ab(a_sample, b_sample, A1, A2, theta_offset, offset):
    """
    对单个采样点，数值搜索theta使模型输出(a,b)最接近采样值
    """
    def error_func(theta):
        a_pred, b_pred = model_signal(theta, A1, A2, theta_offset, offset)
        return (a_sample - a_pred)**2 + (b_sample - b_pred)**2

    res = minimize_scalar(error_func, bounds=(0, 2*np.pi), method='bounded')
    return res.x

if __name__ == "__main__":
    N = 500
    theta_true = np.linspace(0, 2*np.pi, N)

    # 真实参数
    A1_true = 2.0
    A2_true = 0.5
    theta_offset_true = 0.3
    offset_true = 1.0

    a = A1_true * np.cos(theta_true) + A2_true * np.cos(9*theta_true + theta_offset_true) + offset_true
    b = A1_true * np.sin(theta_true) + A2_true * np.sin(9*theta_true + theta_offset_true) + offset_true

    noise_level = 0.01
    a_noisy = a + noise_level * np.random.randn(N)
    b_noisy = b + noise_level * np.random.randn(N)

    # 拟合参数
    A1_fit, A2_fit, theta_offset_fit, offset_fit = fit_parameters(theta_true, a_noisy, b_noisy)

    print("拟合参数：")
    print(f"A1 = {A1_fit:.6f}, A2 = {A2_fit:.6f}, theta_offset = {theta_offset_fit:.6f}, offset = {offset_fit:.6f}")

    # 估计theta
    theta_estimated = np.zeros(N)
    for i in range(N):
        theta_estimated[i] = estimate_theta_from_ab(a_noisy[i], b_noisy[i], A1_fit, A2_fit, theta_offset_fit, offset_fit)

    # 计算误差（角度差异，考虑周期性）
    angle_diff = np.angle(np.exp(1j*(theta_estimated - theta_true)))

    print(f"theta估计误差均值（弧度）：{np.mean(np.abs(angle_diff)):.6e}")

    # 绘图比较
    plt.figure(figsize=(12, 6))
    plt.plot(theta_true, label='真实 theta')
    plt.plot(theta_estimated, label='估计 theta', linestyle='--')
    plt.xlabel('采样点索引')
    plt.ylabel('Theta (rad)')
    plt.title('真实与估计的 Theta 比较')
    plt.legend()
    plt.grid(True)
    plt.show()