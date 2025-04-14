import matplotlib.pyplot as plt
import numpy as np

data = np.random.rand(10, 10)  # 10x10 matrix of random values
plt.imshow(data, cmap='viridis')  # Display as image
plt.colorbar()  # Add a color scale
# plt.show()