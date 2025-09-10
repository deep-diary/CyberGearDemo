import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
from matplotlib.animation import FuncAnimation

class SplineTrajectory:
    """
    三次样条插值轨迹生成与可视化系统
    功能：生成平滑轨迹、可视化静态/动态路径、支持2D/3D展示
    """
    def __init__(self, control_points=None):
        """
        初始化控制点和样条曲线
        :param control_points: 轨迹控制点，格式[[x0,y0], [x1,y1], ...]
        """
        self.control_points = np.array(control_points) if control_points else None
        self.spline = None
        self.trajectory = None
        
    def generate_spline(self, points):
        """
        生成三次样条插值曲线
        :param points: 控制点坐标
        """
        self.control_points = np.array(points)
        # 计算参数化曲线（累积弦长参数化）
        chord_lengths = np.cumsum(np.sqrt(np.sum(np.diff(points, axis=0)**2, axis=1)))
        t = np.insert(chord_lengths, 0, 0) / chord_lengths[-1]
        
        # 创建三次样条对象
        self.spline = CubicSpline(t, points, bc_type='natural')
        
        # 生成密集轨迹点
        t_new = np.linspace(0, 1, 100)
        self.trajectory = self.spline(t_new)
        return self.trajectory
    
    def plot_static_trajectory(self, title="轨迹可视化"):
        """绘制静态轨迹图"""
        plt.figure(figsize=(10, 6))
        
        # 绘制控制点
        plt.scatter(self.control_points[:, 0], self.control_points[:, 1], 
                    c='red', s=80, zorder=5, label='控制点')
        
        # 绘制样条轨迹
        plt.plot(self.trajectory[:, 0], self.trajectory[:, 1], 
                 'b-', linewidth=2.5, label='三次样条轨迹')
        
        # 添加标注和样式
        plt.title(title, fontsize=14)
        plt.xlabel('X坐标', fontsize=12)
        plt.ylabel('Y坐标', fontsize=12)
        plt.grid(True, linestyle='--', alpha=0.7)
        plt.legend()
        plt.axis('equal')
        plt.tight_layout()
        return plt
    
    def animate_trajectory(self, interval=50):
        """生成轨迹动画"""
        fig, ax = plt.subplots(figsize=(10, 6))
        
        # 初始化元素
        ax.scatter(self.control_points[:, 0], self.control_points[:, 1], 
                   c='red', s=80, zorder=5, label='控制点')
        line, = ax.plot([], [], 'b-', linewidth=2.5, label='运动轨迹')
        point, = ax.plot([], [], 'go', markersize=10, label='当前位置')
        
        # 设置绘图区域
        ax.set_title('轨迹动态演示', fontsize=14)
        ax.set_xlabel('X坐标', fontsize=12)
        ax.set_ylabel('Y坐标', fontsize=12)
        ax.grid(True, linestyle='--', alpha=0.7)
        ax.legend()
        ax.axis('equal')
        
        # 计算动态范围（添加10%边界缓冲）
        x_min, x_max = np.min(self.trajectory[:,0]), np.max(self.trajectory[:,0])
        y_min, y_max = np.min(self.trajectory[:,1]), np.max(self.trajectory[:,1])
        x_margin = (x_max - x_min) * 0.1
        y_margin = (y_max - y_min) * 0.1
        ax.set_xlim(x_min - x_margin, x_max + x_margin)
        ax.set_ylim(y_min - y_margin, y_max + y_margin)
        
        def init():
            line.set_data([], [])
            point.set_data([], [])
            return line, point
        
        def update(frame):
            # 更新轨迹线（逐步显示）
            line.set_data(self.trajectory[:frame, 0], 
                         self.trajectory[:frame, 1])
            
            # 更新当前位置点
            if frame < len(self.trajectory):
                point.set_data(self.trajectory[frame, 0], 
                              self.trajectory[frame, 1])
            return line, point
        
        ani = FuncAnimation(fig, update, frames=len(self.trajectory)+10, 
                            init_func=init, blit=True, interval=interval)
        return ani

# ===================== 测试案例 =====================
if __name__ == "__main__":
    # 测试案例1：正弦波路径
    print("测试案例1：正弦波路径")
    angles = np.linspace(0, 2*np.pi, 6)
    sine_points = np.column_stack((angles, np.sin(angles)))
    
    # 生成并可视化轨迹
    sine_traj = SplineTrajectory()
    sine_traj.generate_spline(sine_points)
    plt1 = sine_traj.plot_static_trajectory("正弦波轨迹插值")
    plt1.savefig('sine_trajectory.png')
    plt1.show()
    
    # 保存动画
    sine_ani = sine_traj.animate_trajectory()
    sine_ani.save('sine_animation.gif', writer='pillow')
    
    # 测试案例2：随机路径
    print("\n测试案例2：随机路径")
    np.random.seed(42)
    random_points = np.random.rand(8, 2) * 10
    
    # 生成并可视化轨迹
    random_traj = SplineTrajectory()
    random_traj.generate_spline(random_points)
    plt2 = random_traj.plot_static_trajectory("随机路径插值")
    plt2.savefig('random_trajectory.png')
    plt2.show()
    
    # 保存动画
    random_ani = random_traj.animate_trajectory(interval=30)
    random_ani.save('random_animation.gif', writer='pillow')
    
    print("程序执行完成！已生成以下文件：")
    print("- sine_trajectory.png：静态正弦轨迹图")
    print("- sine_animation.gif：正弦轨迹动画")
    print("- random_trajectory.png：静态随机轨迹图")
    print("- random_animation.gif：随机轨迹动画")