import matplotlib.pyplot as plt
import numpy as np

# Data for left wheel
left_pos_slopes = [0.067429, 0.066914, 0.066988]
left_pos_intercepts = [0.057601, 0.061065, 0.059937]
left_neg_slopes = [0.062087, 0.061285, 0.061606]
left_neg_intercepts = [-0.074771, -0.077566, -0.074816]

# Data for right wheel
right_pos_slopes = [0.063655, 0.064401, 0.063577]
right_pos_intercepts = [0.072272, 0.071084, 0.070016]
right_neg_slopes = [0.072221, 0.071545, 0.07148]
right_neg_intercepts = [-0.057288, -0.058416, -0.055332]

# Calculate mean and standard deviation for left wheel slopes and intercepts
left_pos_slopes_mean = np.mean(left_pos_slopes)
left_pos_slopes_std = np.std(left_pos_slopes)
left_pos_intercepts_mean = np.mean(left_pos_intercepts)
left_pos_intercepts_std = np.std(left_pos_intercepts)

left_neg_slopes_mean = np.mean(left_neg_slopes)
left_neg_slopes_std = np.std(left_neg_slopes)
left_neg_intercepts_mean = np.mean(left_neg_intercepts)
left_neg_intercepts_std = np.std(left_neg_intercepts)

# Calculate mean and standard deviation for right wheel slopes and intercepts
right_pos_slopes_mean = np.mean(right_pos_slopes)
right_pos_slopes_std = np.std(right_pos_slopes)
right_pos_intercepts_mean = np.mean(right_pos_intercepts)
right_pos_intercepts_std = np.std(right_pos_intercepts)

right_neg_slopes_mean = np.mean(right_neg_slopes)
right_neg_slopes_std = np.std(right_neg_slopes)
right_neg_intercepts_mean = np.mean(right_neg_intercepts)
right_neg_intercepts_std = np.std(right_neg_intercepts)

# Print the results
print("Left Wheel:")
print(f"Positive Slopes: Mean = {left_pos_slopes_mean:.6f}, Std = {left_pos_slopes_std:.6f}")
print(f"Positive Intercepts: Mean = {left_pos_intercepts_mean:.6f}, Std = {left_pos_intercepts_std:.6f}")
print(f"Negative Slopes: Mean = {left_neg_slopes_mean:.6f}, Std = {left_neg_slopes_std:.6f}")
print(f"Negative Intercepts: Mean = {left_neg_intercepts_mean:.6f}, Std = {left_neg_intercepts_std:.6f}")

print("\nRight Wheel:")
print(f"Positive Slopes: Mean = {right_pos_slopes_mean:.6f}, Std = {right_pos_slopes_std:.6f}")
print(f"Positive Intercepts: Mean = {right_pos_intercepts_mean:.6f}, Std = {right_pos_intercepts_std:.6f}")
print(f"Negative Slopes: Mean = {right_neg_slopes_mean:.6f}, Std = {right_neg_slopes_std:.6f}")
print(f"Negative Intercepts: Mean = {right_neg_intercepts_mean:.6f}, Std = {right_neg_intercepts_std:.6f}")

# Combined speed range
speeds_neg = np.linspace(-1, 0, 100)
speeds_pos = np.linspace(0, 1, 100)

# Colors per run
colors = ['tab:blue', 'tab:orange', 'tab:green']

# Create subplots
fig, axs = plt.subplots(nrows=2, ncols=1, figsize=(6, 8))
#fig.suptitle("Wheel Speed Calibration")

# Left wheel plot
for i in range(3):
    # Speeds and PWM values for negative and positive parts
    speeds_full = np.concatenate((speeds_neg, speeds_pos))
    pwm_neg = left_neg_slopes[i] * speeds_neg + left_neg_intercepts[i]
    pwm_pos = left_pos_slopes[i] * speeds_pos + left_pos_intercepts[i]
    pwm_full = np.concatenate((pwm_neg, pwm_pos))
    
    axs[0].plot(speeds_full, pwm_full, color=colors[i], label=f'Routine {i+1}')
    
axs[0].set_title("Left Wheel")
axs[0].set_xlabel("Motor Speed (rad/s)")
axs[0].set_ylabel("PWM Duty Cycle")
axs[0].grid(True)
axs[0].legend()

# Right wheel plot
for i in range(3):
    speeds_full = np.concatenate((speeds_neg, speeds_pos))
    pwm_neg = right_neg_slopes[i] * speeds_neg + right_neg_intercepts[i]
    pwm_pos = right_pos_slopes[i] * speeds_pos + right_pos_intercepts[i]
    pwm_full = np.concatenate((pwm_neg, pwm_pos))
    
    axs[1].plot(speeds_full, pwm_full, color=colors[i], label=f'Routine {i+1}')
    
axs[1].set_title("Right Wheel")
axs[1].set_xlabel("Motor Speed (rad/s)")
axs[1].set_ylabel("PWM Duty Cycle")
axs[1].grid(True)
axs[1].legend()

plt.tight_layout()
plt.show()
