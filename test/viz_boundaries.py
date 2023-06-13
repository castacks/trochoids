import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

PATH = "/home/sagar/ipp_ws/src/trochoids/test/csv_files/wind_trochoid.csv"

# Write a function that plots the first column against the second column in a scatter plot
def plot_scatter(x, y):
    plt.scatter(x, y)
    plt.show()

if __name__ == "__main__":
    df = pd.read_csv(PATH)
    # separate the first column from the rest of the dataframe
    x = df.iloc[:, 0]

    seg1 = df.iloc[:,2]
    seg2 = df.iloc[:,3]
    seg3 = df.iloc[:,4]

    rsl_group = []
    rsr_group = []
    rlr_group = []
    lsl_group = []
    lsr_group = []
    lrl_group = []

    segarr1 = []
    segarr2 = []
    segarr3 = []
    segarr4 = []
    segarr5 = []
    segarr6 = []



    # separate the second column from the rest of the dataframe
    y = df.iloc[:, 1]
    # list from 0
    for i in range(len(x)):
        color = 'grey'
        if seg1[i] == 0 and seg2[i] == 2 and seg3[i] == 0:
            color = 'red' #lrl])
            segarr1.append([x[i], y[i]])
            # print('segarr1: ',segarr1)
        elif seg1[i] == 0 and seg2[i] == 1 and seg3[i] == 0:
            color = 'blue' #lsl
            segarr2.append([x[i], y[i]])
            # print('segarr2: ',segarr2)
        elif seg1[i] == 0 and seg2[i] == 1 and seg3[i] == 2:
            color = 'green' #lsr
            segarr3.append([x[i], y[i]])
            # print('segarr3: ',segarr3)
        elif seg1[i] == 2 and seg2[i] == 1 and seg3[i] == 2:
            color = 'purple' #rsr
            segarr4.append([x[i], y[i]])
            # print('segarr4: ',segarr4)
        elif seg1[i] == 2 and seg2[i] == 1 and seg3[i] == 0:
            color = 'yellow' #rsl
            segarr5.append([x[i], y[i]])
            # print('segarr5: ',segarr5)
        elif seg1[i] == 2 and seg2[i] == 0 and seg3[i] == 2:
            color = 'black' #rlr
            segarr6.append([x[i], y[i]])
            print('segarr6: ',segarr6)

        
    # Create a scatter plot of each segarr group in the same plot with the desired colors
    # plt.scatter(segarr1[0], segarr1[1], color='red')
    # print('segarr2: ',segarr2)
    
    segarr1x = []
    segarr1y = []
    segarr2x = []
    segarr2y = []
    segarr3x = []
    segarr3y = []
    segarr4x = []
    segarr4y = []
    segarr5x = []
    segarr5y = []
    segarr6x = []
    segarr6y = []

    
    # Print all the x values of segarr1 which is an array of arrays
    for i in range(len(segarr1)):
        segarr1x.append(segarr1[i][0])
        segarr1y.append(segarr1[i][1])

    plt.scatter(segarr1x, segarr1y, color='red')

    # Print all the x values of segarr2 which is an array of arrays
    for i in range(len(segarr2)):
        segarr2x.append(segarr2[i][0])
        segarr2y.append(segarr2[i][1])
    
    plt.scatter(segarr2x, segarr2y, color='blue')

    # Print all the x values of segarr3 which is an array of arrays
    for i in range(len(segarr3)):
        segarr3x.append(segarr3[i][0])
        segarr3y.append(segarr3[i][1])

    plt.scatter(segarr3x, segarr3y, color='green')

    # Print all the x values of segarr4 which is an array of arrays
    for i in range(len(segarr4)):
        segarr4x.append(segarr4[i][0])
        segarr4y.append(segarr4[i][1])
    
    plt.scatter(segarr4x, segarr4y, color='purple')

    # Print all the x values of segarr5 which is an array of arrays
    for i in range(len(segarr5)):
        segarr5x.append(segarr5[i][0])
        segarr5y.append(segarr5[i][1])
    
    plt.scatter(segarr5x, segarr5y, color='yellow')

    # Print all the x values of segarr6 which is an array of arrays
    for i in range(len(segarr6)):
        segarr6x.append(segarr6[i][0])
        segarr6y.append(segarr6[i][1])
    
    plt.scatter(segarr6x, segarr6y, color='black')
    plt.axis('equal')
    plt.grid()
    plt.show()



    #  


    # plt.scatter(segarr2[0,:], segarr2[1,:], color='blue')
    # plt.scatter(segarr3[0,:], segarr3[1,:], color='green')
    # plt.scatter(segarr4[0,:], segarr4[1,:], color='purple')
    # plt.scatter(segarr5[0,:], segarr5[1,:], color='yellow')
    # # plt.scatter(segarr6[0], segarr6[1], color='black')
    # plt.axis('equal')
    # plt.grid()
    # plt.show()



    # x = df["x"]
    # y = df["y"]
    # plot_scatter(x, y)
