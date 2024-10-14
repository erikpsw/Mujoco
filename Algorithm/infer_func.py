import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

def init_K(K,n):
    """
    initalize the convariance matrix K
    """
    for r in range(K.shape[0]):
        var=K[r,r]
        for c in range(K.shape[1]):
            dist=abs(c-r)
            if 0<dist<n:
                K[c,r]=var*(1-dist/n)
    return K

def gen_C(X,idx):
    """
    initalize the linear observation matrix C
    """
    C=np.zeros_like(X)
    C[idx]=1
    C=C[:,np.newaxis]
    return C