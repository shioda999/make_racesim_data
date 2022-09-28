import numpy as np
def main():
    path = "action\\austria_0925_00h53m.npz"
    data = np.load(path, allow_pickle=True)
    print([k for k in data])
    for k in data:
        print(k, data[k].dtype, data[k].shape)
main()