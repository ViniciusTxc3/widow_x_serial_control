
from ctrl_widow_x import widow_x as wx



def main():
    
    wx.connect()

    x_inicial = 2048
    y_inicial = 250
    z_inicial = 225
    step = 20

    if wx.isConnected:
        for i in range(3):
            x = x_inicial + step*i
            y = y_inicial + step*i
            z = z_inicial + step*i
            wx.sendValue(x,y,z)



if __name__ == "__main__":

 	main()
