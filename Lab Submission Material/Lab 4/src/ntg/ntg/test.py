import numpy as np

a = np.expand_dims(np.array([np.NAN,np.NAN]),axis=0)
b = np.expand_dims(np.arange(a.shape[-1]),axis=0)
print(a.shape)
print(b.shape)
print(np.concatenate((a,b),0))
