import numpy as np

inputs = [0.15,-0.1,0.25] 
position = [0.153, -0.101, 0.249]
print(np.sqrt((inputs[0]- position[0])**2 + (inputs[1]- position[1])**2 + (inputs[2] - position[2])**2))