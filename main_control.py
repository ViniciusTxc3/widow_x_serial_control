
from ctrl_widow_x import widow_x



def main():
    wx = widow_x()
    wx.connect()

    x_inicial = 1700
    y_inicial = 274
    z_inicial = 91
    step = 20
    if wx.isConnected:
        for i in range(1):
            x = x_inicial
            y = y_inicial
            z = z_inicial
            wx.sendValue(x,y,z)



if __name__ == "__main__":

 	main()
