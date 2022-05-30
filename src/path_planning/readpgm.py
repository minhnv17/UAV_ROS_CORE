from PIL import Image
import numpy as np

def read_img(name):
    im = Image.open(name)    #  read a file 
    return np.array(im)
    
if __name__ == '__main__':
    data = read_img('/home/minh/Documents/Navigation_UAV/path_planning/maps/newmap.pgm')
    for i in data:
        for j in i:
            if j == 254:
                print(j)