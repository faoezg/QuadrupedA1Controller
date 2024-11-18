import numpy as np
import matplotlib.pyplot as plt
from scipy.special import comb

def bezier_curve(t, control_points):
    """
    Calculate the Bézier curve at time t using the control points.
    Parameters:
        t (float): The parameter along the curve (0 <= t <= 1).
        control_points (list of tuples): List of control points [(x0, y0), (x1, y1), ...].
    Returns:
        (float, float): The (x, y) coordinates on the Bézier curve at time t.
    """
    n = len(control_points) - 1
    x, y = 0, 0
    for i, (px, py) in enumerate(control_points):
        # Binomial coefficient * control point * (1 - t)^(n - i) * t^i
        bernstein = comb(n, i) * (1 - t)**(n - i) * t**i
        x += bernstein * px
        y += bernstein * py

    return x, y

def generate_bezier_points(control_points, num_points=100):
    """
    Generate points on the Bézier curve using the control points.
    Parameters:
        control_points (list of tuples): List of control points [(x0, y0), (x1, y1), ...].
        num_points (int): Number of points to generate along the curve.
    Returns:
        (np.ndarray, np.ndarray): Arrays of x and y coordinates along the curve.
    """
    t_values = np.linspace(0, 1, num_points)
    x_vals, y_vals = [], []
    for t in t_values:
        x, y = bezier_curve(t, control_points)
        x_vals.append(x)
        y_vals.append(y)
    return np.array(x_vals), np.array(y_vals)

    
# Define the 12 (actually 10) control points based on data table from https://www.mdpi.com/2076-3417/9/7/1508
# 
# (remove P6 and P4) and multiply all points by a velocity command in order to make the trajectory wider. 
# the robot will (hopefully) smooothly transition between standing and trotting
def get_control_points(V_desire, T_sw):

    abs_V_desire = np.abs(V_desire)  # the step high should be the same for either direction

    x_coords = [V_desire/1000 * -170, 
                V_desire/1000 * -(170 + abs_V_desire / ((12 + 1) * T_sw)), 
                V_desire/1000 * -300,
                V_desire/1000 * -300, 
                0, 
                0, 
                V_desire/1000 * 300, 
                V_desire/1000 * 300, 
                V_desire/1000 * (170 + abs_V_desire / ((12 + 1) * T_sw)), 
                V_desire/1000 * 170]
    
    # Y coordinates


    y_coords = [(-270+270)*(abs_V_desire/1000), 
                (-270+270)*(abs_V_desire/1000), 
                (-160+270)*(abs_V_desire/1000), 
                (-160+270)*(abs_V_desire/1000), 
                (-160+270)*(abs_V_desire/1000), 
                (-120+270)*(abs_V_desire/1000), 
                (-120+270)*(abs_V_desire/1000), 
                (-120+270)*(abs_V_desire/1000), 
                (-270+270)*(abs_V_desire/1000), 
                (-270+270)*(abs_V_desire/1000)] if V_desire!=0 else 10 * [0]  # to avoid dividing by zero in case of no movement!
    
    return list(zip(x_coords, y_coords))

# Plot Bézier curve for different values of V_desire
def plot_bezier_curve(V_desires, T_sw=1.0):
    plt.figure(figsize=(10, 6))
    for V_desire in V_desires:
        control_points = get_control_points(-V_desire, T_sw)
        print(len(control_points))
        x_vals, y_vals = generate_bezier_points(control_points)
        plt.plot(x_vals, y_vals, label=f'V_desire = {V_desire}')
        
        # Plot control points for each V_desire
        plt.scatter(*zip(*control_points), label=f'Control Points (V_desire={V_desire})', s=20)
    
    plt.xlabel("X (mm)")
    plt.ylabel("Y (mm)")
    plt.title("Bézier Curve for Swing Phase with Different Desired Velocities")
    plt.legend()
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    # Example usage: Plot for different desired velocities
    V_desires = [1000]  # mm/s
    plot_bezier_curve(V_desires)
