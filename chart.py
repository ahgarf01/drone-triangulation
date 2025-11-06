import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import TextBox
import ast

# -------------------------
# Settings
# -------------------------
avg_aoa_error_deg = 5.0
use_log_scale = True
min_clip_error = 1e-6
max_display_error = 500
num_crosses = 100

# -------------------------
# Initial station coordinates
# -------------------------
stations = [(-50, 0), (50, 0), (0, 87)]
station_scatter = None
rays = []
intersection_cloud = None


# -------------------------
# Weighting and Estimation Functions
# -------------------------
def calculate_standard_weights(stations, p_k):
    """Calculates weights using the standard 1/distance^2 method."""
    distances_sq = np.sum((np.array(stations) - p_k)**2, axis=1)
    weights = 1.0 / (distances_sq + 1e-9)
    return weights

def estimate_intersection(stations, angles, max_iter=10, tol=1e-4):
    """
    Estimates the intersection of rays using standard IWLS.
    """
    num_stations = len(stations)
    if num_stations < 2: return np.nan, np.nan
    A = np.zeros((num_stations, 2)); b = np.zeros(num_stations)
    for i, ((sx, sy), angle) in enumerate(zip(stations, angles)):
        cos_a, sin_a = math.cos(angle), math.sin(angle)
        A[i, 0] = -sin_a; A[i, 1] = cos_a
        b[i] = -sx * sin_a + sy * cos_a
    try: p_k = np.linalg.pinv(A) @ b
    except np.linalg.LinAlgError: return np.nan, np.nan

    for _ in range(max_iter):
        weights = calculate_standard_weights(stations, p_k)
        W = np.diag(weights)
        try:
            A_T_W = A.T @ W
            p_next = np.linalg.inv(A_T_W @ A) @ A_T_W @ b
        except np.linalg.LinAlgError: return p_k[0], p_k[1]
        if np.linalg.norm(p_next - p_k) < tol: return p_next[0], p_next[1]
        p_k = p_next
    return p_k[0], p_k[1]

def triangulation_error_point(x, y, stations, aoa_error_deg=5.0):
    sigma_theta = math.radians(aoa_error_deg)
    w = 1.0 / (sigma_theta ** 2)
    F11, F12, F22 = 0.0, 0.0, 0.0
    for xi, yi in stations:
        dx, dy = x - xi, y - yi
        r2 = dx**2 + dy**2
        if r2 < 1e-12: return max_display_error
        h_x, h_y = -dy / r2, dx / r2
        F11 += h_x**2; F12 += h_x*h_y; F22 += h_y**2
    F11 *= w; F12 *= w; F22 *= w
    tr = F11 + F22
    det = max(F11*F22 - F12*F12, 0.0)
    discr = max(tr*tr - 4*det, 0.0)
    eig_min = 0.5 * (tr - math.sqrt(discr))
    if eig_min <= 0: return max_display_error
    return min(math.sqrt(1.0 / eig_min), max_display_error)

# -------------------------
# Plotting and GUI Functions
# -------------------------
def compute_error_grid_vectorized(xmin, xmax, ymin, ymax, resolution):
    xs, ys = np.array([s[0] for s in stations]), np.array([s[1] for s in stations])
    x, y = np.linspace(xmin, xmax, resolution), np.linspace(ymin, ymax, resolution)
    X, Y = np.meshgrid(x, y)
    F11, F12, F22 = np.zeros_like(X), np.zeros_like(X), np.zeros_like(X)
    sigma_theta = math.radians(avg_aoa_error_deg)
    w = 1.0 / (sigma_theta * sigma_theta)
    too_close_mask = np.zeros_like(X, dtype=bool)
    for xi, yi in zip(xs, ys):
        dx, dy = X - xi, Y - yi
        r2 = dx*dx + dy*dy
        too_close_mask |= (r2 < 1e-6)
        safe_r2 = np.where(r2 < 1e-6, np.nan, r2)
        h_x, h_y = -dy / safe_r2, dx / safe_r2
        F11 += h_x*h_x; F12 += h_x*h_y; F22 += h_y*h_y
    F11 *= w; F12 *= w; F22 *= w
    tr = F11 + F22
    det = np.maximum(F11*F22 - F12*F12, 0.0)
    discr = np.maximum(tr*tr - 4.0*det, 0.0)
    eig_min = 0.5*(tr - np.sqrt(discr))
    ill_mask = (~np.isfinite(eig_min)) | (eig_min <= 0) | too_close_mask
    with np.errstate(divide='ignore', invalid='ignore'):
        Z_actual = np.sqrt(1.0 / eig_min)
    Z_actual[ill_mask] = np.inf
    Z_actual = np.minimum(Z_actual, max_display_error)
    return X, Y, Z_actual

def adaptive_resolution(xmin, xmax, ymin, ymax, base_res=150, reference_span=4000.0,
                        min_res=60, max_res=400):
    max_span = max(xmax - xmin, ymax - ymin, 1e-9)
    res = int(base_res * (reference_span / max_span))
    return max(min_res, min(max_res, res))

def update_plot(event=None):
    global cbar, station_scatter
    xmin, xmax = ax.get_xlim(); ymin, ymax = ax.get_ylim()
    res = adaptive_resolution(xmin, xmax, ymin, ymax)
    X, Y, Z_actual = compute_error_grid_vectorized(xmin, xmax, ymin, ymax, res)
    finite_vals = Z_actual[np.isfinite(Z_actual) & (Z_actual > 0)]
    Z_min, Z_max = (finite_vals.min(), finite_vals.max()) if finite_vals.size > 0 else (min_clip_error, max_display_error)

    if use_log_scale:
        Z_pos = np.where(Z_actual > 0, Z_actual, np.nan)
        Z_display = np.log10(np.clip(Z_pos, Z_min, Z_max))
        vmin, vmax = np.log10(Z_min), np.log10(Z_max)
    else:
        Z_display = np.clip(Z_actual, Z_min, Z_max)
        vmin, vmax = Z_min, Z_max

    for coll in getattr(update_plot, "artists", []):
        try: coll.remove()
        except: pass
    update_plot.artists = []
    contourf = ax.contourf(X, Y, Z_display, levels=40, cmap='viridis', vmin=vmin, vmax=vmax, extend='both')
    update_plot.artists.append(contourf)

    if 'cbar' in globals() and cbar is not None: cbar.ax.remove()
    cbar_ax = fig.add_axes([0.92, 0.15, 0.02, 0.7])
    cbar = fig.colorbar(contourf, cax=cbar_ax)

    if use_log_scale:
        tick_locs = cbar.get_ticks()
        cbar.set_ticks(tick_locs)
        cbar.set_ticklabels([f"{10**t:.0f}" if 10**t >= 1 else f"{10**t:.2g}" for t in tick_locs])
        cbar.set_label("Error [m] (actual)")
    else: cbar.set_label("Error [m]")

    if finite_vals.size > 0:
        levels = np.logspace(np.log10(Z_min), np.log10(Z_max), 8) if use_log_scale else np.linspace(Z_min, Z_max, 8)
        contour_lines = ax.contour(X, Y, Z_actual, levels=levels, colors='k', linewidths=0.7)
        ax.clabel(contour_lines, inline=True, fontsize=8, fmt="%.0f m", inline_spacing=1)
        update_plot.artists.append(contour_lines)

    if 'station_scatter' in globals() and station_scatter is not None: station_scatter.remove()
    station_xs, station_ys = zip(*stations) if stations else ([], [])
    station_scatter = ax.scatter(station_xs, station_ys, color='red', s=80, edgecolors='white', linewidth=1.2, zorder=5)
    
    ax.set_xlim(xmin, xmax); ax.set_ylim(ymin, ymax)
    ax.set_xlabel("X position of drone (m)"); ax.set_ylabel("Y position of drone (m)")
    ax.set_title(f"Triangulation Error Contour Map\nAoA Error={avg_aoa_error_deg}°")
    fig.canvas.draw_idle()

# --- MODIFIED ---
def on_click(event):
    if event.inaxes != ax or event.button != 1: return

    x_click, y_click = event.xdata, event.ydata
    
    # Calculate error at the clicked point
    error_val = triangulation_error_point(x_click, y_click, stations)
    error_str = f"Error ≈ {error_val:.1f} m" if np.isfinite(error_val) else "Error ≈ ∞"
    
    # Calculate distance to the centroid of the stations
    centroid = np.mean(stations, axis=0) if stations else [0,0]
    dist_to_center = np.hypot(x_click - centroid[0], y_click - centroid[1])
    center_dist_str = f"Dist to center ≈ {dist_to_center:.1f} m"
    
    # Calculate distance to the nearest station
    if stations:
        distances_to_stations = [np.hypot(x_click - sx, y_click - sy) for sx, sy in stations]
        min_dist = min(distances_to_stations)
        nearest_dist_str = f"Dist to nearest ≈ {min_dist:.1f} m"
    else:
        nearest_dist_str = "Dist to nearest ≈ N/A"

    # Update the text box with all three fields
    error_text.set_text(f"{error_str}\n{center_dist_str}\n{nearest_dist_str}")
    fig.canvas.draw_idle()
# --- END MODIFIED ---

def on_motion(event):
    global intersection_cloud
    if event.inaxes != ax:
        if intersection_cloud:
            intersection_cloud.set_offsets(np.empty((0, 2)))
            fig.canvas.draw_idle()
        return

    if not all([event.xdata, event.ydata, len(stations) == len(rays)]):
        return

    x_cursor, y_cursor = event.xdata, event.ydata
    estimated_points, last_noisy_angles = [], []

    for i in range(num_crosses):
        noisy_angles = []
        for sx, sy in stations:
            true_angle = math.atan2(y_cursor - sy, x_cursor - sx)
            angle_error_rad = np.random.normal(scale=math.radians(avg_aoa_error_deg))
            noisy_angles.append(true_angle + angle_error_rad)
        if i == num_crosses - 1: last_noisy_angles = noisy_angles
        est_x, est_y = estimate_intersection(stations, noisy_angles)
        if not np.isnan(est_x): estimated_points.append([est_x, est_y])

    ray_length = 1e7
    for i, ray in enumerate(rays):
        sx, sy = stations[i]
        x_end = sx + ray_length * math.cos(last_noisy_angles[i])
        y_end = sy + ray_length * math.sin(last_noisy_angles[i])
        ray.set_data([sx, x_end], [sy, y_end]); ray.set_alpha(0.7)

    if estimated_points: intersection_cloud.set_offsets(np.array(estimated_points))
    else: intersection_cloud.set_offsets(np.empty((0, 2)))
    fig.canvas.draw_idle()

def submit_stations(text):
    global stations, rays
    try:
        coords = ast.literal_eval(text)
        if isinstance(coords, (list, tuple)):
            stations = [tuple(map(float, xy)) for xy in coords]
            for ray in rays: ray.remove()
            rays.clear()
            for _ in stations:
                line, = ax.plot([], [], 'k--', lw=0.8, alpha=0.7, zorder=4)
                rays.append(line)
            update_plot()
    except Exception as e: print("Error parsing stations:", e)

def on_scroll(event):
    if not event.inaxes or not event.xdata or not event.ydata: return
    scale_factor = 1.2 if event.button == 'down' else 1/1.2
    cur_xlim, cur_ylim = ax.get_xlim(), ax.get_ylim()
    xdata, ydata = event.xdata, event.ydata
    new_xlim = [xdata - (xdata - cur_xlim[0]) * scale_factor, xdata + (cur_xlim[1] - xdata) * scale_factor]
    new_ylim = [ydata - (ydata - cur_ylim[0]) * scale_factor, ydata + (cur_ylim[1] - ydata) * scale_factor]
    ax.set_xlim(new_xlim); ax.set_ylim(new_ylim)
    update_plot()

# --- Main Script Execution ---
fig, ax = plt.subplots(figsize=(10, 8))
ax.set_aspect('equal', adjustable='box')
plt.subplots_adjust(left=0.1, right=0.9, top=0.85)
update_plot.artists = []
error_text = ax.text(0.95, 0.05, "", transform=ax.transAxes, fontsize=10, ha='right', va='bottom',
                     bbox=dict(facecolor='white', alpha=0.7, edgecolor='none'))
axbox = plt.axes([0.15, 0.92, 0.7, 0.05])
station_text = TextBox(axbox, "Stations: ", initial=str(stations))
for _ in stations:
    line, = ax.plot([], [], 'k--', lw=0.8, alpha=0.7, zorder=4)
    rays.append(line)
intersection_cloud = ax.scatter([], [], c='c', marker='+', alpha=0.5, zorder=10)

# Connect events
fig.canvas.mpl_connect('button_press_event', on_click)
fig.canvas.mpl_connect('motion_notify_event', on_motion)
station_text.on_submit(submit_stations)
fig.canvas.mpl_connect('scroll_event', on_scroll)
fig.canvas.mpl_connect("button_release_event", update_plot)

# Initial plot
ax.set_xlim(-2000, 2000); ax.set_ylim(-2000, 2000)
update_plot()
plt.show()