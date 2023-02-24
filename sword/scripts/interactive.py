# %%
import numpy as np
pos = np.array([32.34314082, 113.32202031, -115.34423359])
pos2 = np.array([30.44523811, 90.67622375, -124.00296021])
bbx = np.array([36.60995851, 36.94077546, 33.37921779])
center = np.array([0, 0.19607843, 0.11711027])
print(pos - pos2)
print((pos - pos2) / bbx)
print((pos - pos2) / bbx + center)
# %%
