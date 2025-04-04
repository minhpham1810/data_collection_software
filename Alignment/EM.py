import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from scipy.optimize import minimize

# Generate test signal
def generate_test_signal(pattern_func, pattern_length, num_repeats, gaps):
    """Generate signal with repeated patterns and specified gaps"""
    signal = []
    starts = []
    t = 0
    for i in range(num_repeats):
        starts.append(t)
        signal.extend(pattern_func(np.linspace(0, 1, pattern_length)))
        if i < len(gaps):
            signal.extend(np.zeros(gaps[i]))
            t += pattern_length + gaps[i]
        else:
            t += pattern_length
    return np.array(signal), np.array(starts)

# Extract phi_i(t) using interpolation and segment-specific start/end
def extract_variable_segments(phi, t_start, t_end, pattern_length):
    interp_func = interp1d(np.arange(len(phi)), phi, kind='linear', bounds_error=False, fill_value=0.0)
    phi_i_list = []
    for i in range(len(t_start)):
        duration = t_end[i] - t_start[i]
        if duration <= 0:
            phi_i_list.append(np.zeros(pattern_length))
            continue
        segment_time = np.linspace(t_start[i], t_end[i], pattern_length)
        phi_segment = interp_func(segment_time)
        phi_i_list.append(phi_segment)
    return np.array(phi_i_list)

# Expectation step: average aligned segments
def expectation_variable(phi, t_start, t_end, pattern_length):
    phi_i = extract_variable_segments(phi, t_start, t_end, pattern_length)
    return np.mean(phi_i, axis=0)

# Cost function: error between x(t) and average of phi_i
def cost_variable(params, phi, K, pattern_length, x_t):
    if not np.all(np.isfinite(params)):
        return np.inf

    t_start = params[:K]
    t_end = params[K:]

    if np.any(t_end - t_start <= 1):
        return np.inf

    all_times = np.empty(2 * K)
    all_times[0::2] = t_start
    all_times[1::2] = t_end

    if np.any(np.diff(all_times) <= 0):
        return np.inf

    phi_i = extract_variable_segments(phi, t_start, t_end, pattern_length)
    reconstruction = np.mean(phi_i, axis=0)
    if np.any(~np.isfinite(reconstruction)):
        return np.inf

    return np.sum((x_t - reconstruction) ** 2)

# EM algorithm with variable segment lengths
def em_variable_segments(phi, pattern_length, K, max_iter=50):
    total_length = len(phi)
    segment_size = pattern_length + 10  # small gap buffer
    t_start = np.linspace(0, total_length - segment_size, K)
    t_end = t_start + pattern_length

    for _ in range(max_iter):
        x_t = expectation_variable(phi, t_start, t_end, pattern_length)
        init_params = np.concatenate([t_start, t_end])
        bounds = [(0, total_length) for _ in range(2 * K)]

        result = minimize(
            cost_variable,
            init_params,
            args=(phi, K, pattern_length, x_t),
            bounds=bounds,
            method='L-BFGS-B'
        )
        t_start = np.sort(result.x[:K])
        t_end = np.sort(result.x[K:])

        # Ensure ordering t_{1s} < t_{1e} < t_{2s} < ...
        for i in range(K):
            if t_end[i] <= t_start[i]:
                t_end[i] = t_start[i] + 1
            if i > 0 and t_start[i] <= t_end[i - 1]:
                t_start[i] = t_end[i - 1] + 1
                t_end[i] = max(t_end[i], t_start[i] + 1)

    final_x = expectation_variable(phi, t_start, t_end, pattern_length)
    return final_x, t_start, t_end

# Define the sine wave pattern
def sine_pattern(t):
    return np.sin(2 * np.pi * t)

# Define pattern length and gaps
pattern_length = 100
num_repeats = 5
gaps = [30, 50, 40, 60]  # gaps between repeats

# Generate new test signal with gaps
phi, true_t = generate_test_signal(sine_pattern, pattern_length, num_repeats, gaps)

# Run the improved EM algorithm with variable-length segments
estimated_pattern_var, est_start, est_end = em_variable_segments(phi, pattern_length, num_repeats)

# Plotting results
plt.figure(figsize=(14, 6))
plt.subplot(2, 1, 1)
plt.plot(phi)
for s, e in zip(est_start, est_end):
    plt.axvline(s, color='green', linestyle='--', alpha=0.5)
    plt.axvline(e, color='red', linestyle='--', alpha=0.5)
plt.title("Input Signal with Estimated Start (green) and End (red) of Segments")

plt.subplot(2, 1, 2)
plt.plot(estimated_pattern_var, label='Estimated Pattern (Variable Segments)')
plt.plot(sine_pattern(np.linspace(0, 1, pattern_length)), '--', label='True Pattern')
plt.title("Recovered Pattern vs True Pattern (Variable Segments)")
plt.legend()
plt.tight_layout()
plt.show()