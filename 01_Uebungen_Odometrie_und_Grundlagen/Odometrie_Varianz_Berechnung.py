n = 30
x_values = (-0.4, 0.1, -0.3, -0.8, 0, -0.7, -0.3, -0.9, -0.2, -0.7, -0.2, -0.3, 0.4, -0.1, -0.5, -0.5, 0, 0.3, 0.4, 0, -0.4, -1.1, 0.2, 0.3, 0, 0, 0.3, -0.2, 0, 0.9)
y_values = (50-0.2, 50+0.1, 50-0.1, 50-0.1, 50+0.4, 49.9, 50, 50, 50.1, 49.7, 49.7, 50, 49.4, 50.1, 49.7, 49.8, 49.9, 49.4, 49.8, 49.7, 49.6, 50.1, 50.2, 50.1, 49.9, 50, 50.6, 49.9, 50.2, 50)

theta_values_r = (89, 88, 87, 92, 90, 91, 90, 89, 91, 88, 90, 92, 89, 91, 90, 88, 91, 89, 90, 92, 88, 90, 91, 89, 90, 88, 92, 91, 89, 90)
theta_values_l = (91, 89, 92, 92, 88, 91, 90, 89, 92, 91, 88, 88, 91, 89, 93, 90, 88, 91, 90, 92, 89, 91, 90, 88, 92, 91, 89, 90, 91, 88)

x_mean = sum(x_values) / n
y_mean = sum(y_values) / n
theta_r_mean = sum(theta_values_r) / n
theta_l_mean = sum(theta_values_l) / n

x_variance = 0
y_variance = 0
theta_r_variance = 0
theta_l_variance = 0

xy_covariance = 0

for i in range(n):
    xy_covariance += (x_values[i] - x_mean) * (y_values[i] - y_mean)
xy_covariance = xy_covariance / n

for y in y_values:
    y_variance += (y - y_mean) ** 2
y_variance = y_variance / n

for x in x_values:
    x_variance += (x - x_mean) ** 2
x_variance = x_variance / n

for theta_r in theta_values_r:
    theta_r_variance += (theta_r - theta_r_mean) ** 2

for theta_l in theta_values_l:
    theta_l_variance += (theta_l - theta_l_mean) ** 2

theta_r_variance = theta_r_variance / n
theta_l_variance = theta_l_variance / n

print(f"Mean of x: {x_mean}")
print(f"Mean of y: {y_mean}")
print(f"Variance of x: {x_variance}")
print(f"Variance of y: {y_variance}")
print(f"Covariance of x and y: {xy_covariance}")
print(f"Mean of theta (right): {theta_r_mean}")
print(f"Mean of theta (left): {theta_l_mean}")
print(f"Variance of theta (right): {theta_r_variance}")
print(f"Variance of theta (left): {theta_l_variance}")