import h5py

f = h5py.File("activity_cnn.h5", "r")

def walk(name, obj):
    print(name)

f.visititems(walk)
