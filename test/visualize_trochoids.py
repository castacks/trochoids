# ## MO
# import os
# import pandas as pd
# import matplotlib.pyplot as plt

# ### Set your path to the folder containing the .csv files
# PATH = 'csv_files/' # Use your path

# fileNames = os.listdir(PATH)
# fileNames = [file for file in fileNames if '.csv' in file]
# fileNames.sort()
# dataframes_list = []
# files = []
# ### Loop over all files
# for file in fileNames:
#     df = pd.read_csv(PATH + file, sep=',', header=None)
#     dataframes_list.append(df)
#     files.append(file)
    
# for dataframe, file in zip(dataframes_list, files):    
#     file = file.split('.')[0]
#     file = file.replace('_', ' ')
#     file = file.capitalize()

#     plt.scatter(dataframe[0], dataframe[1], label=file)
#     plt.axis('equal')
#     # plt.xlim(-50,150)
#     # plt.ylim(-50,150)
#     # plt.axis('equal')
#     # plt.set_aspect('equal',adjustable='box')




#     # plt.xlim(-10, 110)
#     # plt.ylim(0, 0.04)
#     plt.legend(loc = 'upper left')
#     plt.title(file)
#     plt.grid()
#     plt.savefig('figures/'+ file + '.png', transparent=True)
#     plt.show()

import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

### Set your path to the folder containing the .csv files
PATH = 'csv_files/' # Use your path

# List the CSV files in the directory
csv_files = [file for file in os.listdir(PATH) if file.endswith('.csv')]

# Create an empty dictionary to store the dataframes
dataframes = {}

# Read each CSV file and store it in a dataframe
for file in csv_files:
    file_path = os.path.join(PATH, file)
    df_name = os.path.splitext(file)[0]  # Use the file name as the dataframe name
    file = file.replace('_', ' ')
    dataframes[df_name] = pd.read_csv(file_path)

######################################################################################################################################################################
# def plot_vector_field(ax, vx, vy, x_min, x_max, y_min, y_max, rsr_columnx, rsr_columny, lsr_columnx, lsr_columny, rsl_columnx, rsl_columny, lsl_columnx, lsl_columny):
#     # Define the grid
#     x = np.linspace(x_min - 400, x_max + 400, 15)
#     y = np.linspace(y_min - 400, y_max + 400, 15)
#     X, Y = np.meshgrid(x, y)

#     # Calculate the magnitude and direction of the vectors
#     U = vx
#     V = vy
#     magnitude = np.sqrt(U**2 + V**2)
#     theta = np.arctan2(V, U)

#     lastelementx = rsr_columnx.iloc[-1]
#     lastelementy = rsr_columny.iloc[-1]

#     ax.axis('equal')
#     ax.plot(rsr_columnx[0], rsr_columny[0], 'ko')
#     ax.plot(lastelementx, lastelementy, 'ko')

#     ax.text(rsr_columnx[0], rsr_columny[0], 'Start', fontsize=12)
#     ax.text(lastelementx, lastelementy, 'End', fontsize=12)
    
#     # ax.quiver(X, Y, U, V, angles='xy', color=[0, 0, 0, 0.2], pivot='mid', width=0.003)
#     ax.plot(rsr_columnx, rsr_columny, label='RSR', color=(255/255, 147/255, 30/255))
#     ax.plot(lsr_columnx, lsr_columny, label='LSR', color=(63/255, 169/255, 245/255))
#     ax.plot(rsl_columnx, rsl_columny, label='RSL',color=(255/255, 29/255, 37/255), linewidth=3.0)
#     ax.plot(lsl_columnx, lsl_columny, label='LSL',color=(122/255, 201/255, 67/255))
#     # ax.legend()


# if __name__ == '__main__':
#     vx = 23.2288
#     vy = -4.51207

#     RSR = dataframes['RSR']
#     LSR = dataframes['LSR']
#     RSL = dataframes['RSL']
#     LSL = dataframes['LSL']

#     rsr_columnx = RSR.iloc[:, 0]
#     rsr_columny = RSR.iloc[:, 1]

#     lsr_columnx = LSR.iloc[:, 0]
#     lsr_columny = LSR.iloc[:, 1]

#     rsl_columnx = RSL.iloc[:, 0]
#     rsl_columny = RSL.iloc[:, 1]

#     lsl_columnx = LSL.iloc[:, 0]
#     lsl_columny = LSL.iloc[:, 1]

#     xmin_lsl = min(lsl_columnx)
#     xmax_lsl = max(lsl_columnx)
#     ymin_lsl = min(lsl_columny)
#     ymax_lsl = max(lsl_columny)

#     xmin_lsr = min(lsr_columnx)
#     xmax_lsr = max(lsr_columnx)
#     ymin_lsr = min(lsr_columny)
#     ymax_lsr = max(lsr_columny)

#     xmin_rsl = min(rsl_columnx)
#     xmax_rsl = max(rsl_columnx)
#     ymin_rsl = min(rsl_columny)
#     ymax_rsl = max(rsl_columny)

#     xmin_rsr = min(rsr_columnx)
#     xmax_rsr = max(rsr_columnx)
#     ymin_rsr = min(rsr_columny)
#     ymax_rsr = max(rsr_columny)

#     xmin = min(xmin_lsl, xmin_lsr, xmin_rsl, xmin_rsr)
#     xmax = max(xmax_lsl, xmax_lsr, xmax_rsl, xmax_rsr) 
#     ymin = min(ymin_lsl, ymin_lsr, ymin_rsl, ymin_rsr) 
#     ymax = max(ymax_lsl, ymax_lsr, ymax_rsl, ymax_rsr) 

#     fig, ax = plt.subplots(figsize=(6.4, 4.8))
#     plot_vector_field(ax, vx, vy, xmin, xmax, ymin, ymax, rsr_columnx, rsr_columny, lsr_columnx, lsr_columny, rsl_columnx, rsl_columny, lsl_columnx, lsl_columny)
#     plt.xlabel(r'$x^i$')
#     plt.ylabel(r'$y^i$')
#     plt.show()




#######################################################################################################################################################################
def plot_vector_field(ax, vx, vy, x_min, x_max, y_min, y_max, lsl_columnx, lsl_columny, lsl_columnpsi):
    # Define the grid
    x = np.linspace(x_min, x_max, 10)
    y = np.linspace(y_min, y_max, 10)
    X, Y = np.meshgrid(x, y)

    # Calculate the magnitude and direction of the vectors
    U = vx
    V = vy
    magnitude = np.sqrt(U**2 + V**2)
    theta = np.arctan2(V, U)

    lastelementx = lsl_columnx.iloc[-1]
    lastelementy = lsl_columny.iloc[-1]

    ax.axis('equal')
    ax.plot(lsl_columnx[0], lsl_columny[0], 'ko')
    ax.plot(lastelementx, lastelementy, 'ko')

    # psi_x = np.linspace(np.min(lsl_columnpsi), np.max(lsl_columnpsi), 10)
    # psi_y = np.linspace(np.min(lsl_columnpsi), np.max(lsl_columnpsi), 10)
    # psi_X, psi_Y = np.meshgrid(psi_x, psi_y)


    # Plot a quiver  
    for i in range(len(lsl_columnx)):
        # psi_x = np.linspace(np.min(lsl_columnpsi[i]), np.max(lsl_columnpsi[i]), 10)
        # psi_y = np.linspace(np.min(lsl_columnpsi[i]), np.max(lsl_columnpsi[i]), 10)
        # psi_X, psi_Y = np.meshgrid(psi_x, psi_y)
        plt.quiver(lsl_columnx[i], lsl_columny[i],np.cos(lsl_columnpsi[i]), np.sin(lsl_columnpsi[i]), angles='xy', color=[0, 0, 0, 1], pivot='tail', width=0.001)
        

    ax.text(lsl_columnx[0], lsl_columny[0], 'Start', fontsize=12)
    ax.text(lastelementx, lastelementy, 'End', fontsize=12)

    # ax.quiver(X, Y, U, V, angles='xy', color=[0, 0, 0, 0.2], pivot='mid', width=0.003)

    ax.plot(lsl_columnx, lsl_columny, label='LSL')
    # ax.plot(lsr_columnx, lsr_columny, label='LSR')
    # ax.plot(rsl_columnx, rsl_columny, label='RSL')
    # ax.plot(lsl_columnx, lsl_columny, label='LSL')
    # ax.legend()


if __name__ == '__main__':
    vx = 0.5
    vy = 0.75

    LSL = dataframes['LSL']
    # LSR = dataframes['LSR']
    # RSL = dataframes['RSL']
    # LSL = dataframes['LSL']

    lsl_columnx = LSL.iloc[:, 0]
    lsl_columny = LSL.iloc[:, 1]
    lsl_columnpsi = LSL.iloc[:, 3]

    # lsr_columnx = LSR.iloc[:, 0]
    # lsr_columny = LSR.iloc[:, 1]

    # rsl_columnx = RSL.iloc[:, 0]
    # rsl_columny = RSL.iloc[:, 1]

    # lsl_columnx = LSL.iloc[:, 0]
    # lsl_columny = LSL.iloc[:, 1]

    xmin_lsl = min(lsl_columnx)
    xmax_lsl = max(lsl_columnx)
    ymin_lsl = min(lsl_columny)
    ymax_lsl = max(lsl_columny)

    # xmin_lsr = min(lsr_columnx)
    # xmax_lsr = max(lsr_columnx)
    # ymin_lsr = min(lsr_columny)
    # ymax_lsr = max(lsr_columny)

    # xmin_rsl = min(rsl_columnx)
    # xmax_rsl = max(rsl_columnx)
    # ymin_rsl = min(rsl_columny)
    # ymax_rsl = max(rsl_columny)

    # xmin_rsr = min(lsl_columnx)
    # xmax_rsr = max(lsl_columnx)
    # ymin_rsr = min(lsl_columny)
    # ymax_rsr = max(lsl_columny)

    # xmin = min(xmin_rsr)
    # xmax = max(xmax_rsr) 
    # ymin = min(ymin_rsr) 
    # ymax = max(ymax_rsr) 

    fig, ax = plt.subplots(figsize=(6.4, 4.8))
    plot_vector_field(ax, vx, vy, xmin_lsl, xmax_lsl, ymin_lsl, ymax_lsl, lsl_columnx, lsl_columny, lsl_columnpsi)
    plt.xlabel(r'$x^i$')
    plt.ylabel(r'$y^i$')
    plt.show()


#######################################################################################################################################################################
# def plot_vector_field(ax, vx, vy, x_min, x_max, y_min, y_max, rsr_columnx, rsr_columny, lsr_columnx, lsr_columny):
#     # Define the grid
#     x = np.linspace(x_min - 100, x_max + 100, 30)
#     y = np.linspace(y_min - 100, y_max + 100, 30)
#     X, Y = np.meshgrid(x, y)

#     # Calculate the magnitude and direction of the vectors
#     U = vx
#     V = vy
#     magnitude = np.sqrt(U**2 + V**2)
#     theta = np.arctan2(V, U)

#     lastelementx = rsr_columnx.iloc[-1]
#     lastelementy = rsr_columny.iloc[-1]

#     lastelementlsrx = lsr_columnx.iloc[-1]
#     lastelementlsry = lsr_columny.iloc[-1]

#     ax.axis('equal')
#     ax.plot(rsr_columnx[0], rsr_columny[0], 'ko')
#     ax.plot(lastelementx, lastelementy, 'ko')

#     # ax.plot(lsr_columnx[0], rsr_columny[0], 'ko')
#     ax.plot(lastelementlsrx, lastelementlsry, 'ko')

#     ax.text(rsr_columnx[0], rsr_columny[0], 'Start', fontsize=12)
#     # ax.text(lastelementx, lastelementy, 'End(Dubins)', fontsize=12)
#     # ax.text(lastelementlsrx, lastelementlsry, 'End(Wind)', fontsize=12)

#     ax.quiver(X, Y, U, V, angles='xy', color=[0, 0, 0, 0.2], pivot='mid', width=0.006, scale=200)
#     ax.plot(rsr_columnx, rsr_columny, label='Ground Path')
#     ax.plot(lsr_columnx, lsr_columny, label='Air Frame Path')
#     # ax.plot(rsl_columnx, rsl_columny, label='RSL')
#     # ax.plot(lsl_columnx, lsl_columny, label='LSL')
#     ax.legend()


# if __name__ == '__main__':
#     vx = 10
#     vy = 0

#     RSR = dataframes['RSR']
#     LSR = dataframes['LSR']
#     # RSL = dataframes['RSL']
#     # LSL = dataframes['LSL']

#     rsr_columnx = RSR.iloc[:, 0]
#     rsr_columny = RSR.iloc[:, 1]

#     lsr_columnx = LSR.iloc[:, 0]
#     lsr_columny = LSR.iloc[:, 1]

#     # rsl_columnx = RSL.iloc[:, 0]
#     # rsl_columny = RSL.iloc[:, 1]

#     # lsl_columnx = LSL.iloc[:, 0]
#     # lsl_columny = LSL.iloc[:, 1]

#     # xmin_lsl = min(lsl_columnx)
#     # xmax_lsl = max(lsl_columnx)
#     # ymin_lsl = min(lsl_columny)
#     # ymax_lsl = max(lsl_columny)

#     xmin_lsr = min(lsr_columnx)
#     xmax_lsr = max(lsr_columnx)
#     ymin_lsr = min(lsr_columny)
#     ymax_lsr = max(lsr_columny)

#     # xmin_rsl = min(rsl_columnx)
#     # xmax_rsl = max(rsl_columnx)
#     # ymin_rsl = min(rsl_columny)
#     # ymax_rsl = max(rsl_columny)

#     xmin_rsr = min(rsr_columnx)
#     xmax_rsr = max(rsr_columnx)
#     ymin_rsr = min(rsr_columny)
#     ymax_rsr = max(rsr_columny)


#     xmin = min(xmin_lsr, xmin_rsr)
#     xmax = max(xmax_lsr, xmax_rsr) 
#     ymin = min(ymin_lsr, ymin_rsr) 
#     ymax = max(ymax_lsr, ymax_rsr) 
#     fig, ax = plt.subplots(figsize=(6.4, 4.8))
#     plot_vector_field(ax, vx, vy, xmin, xmax, ymin, ymax, rsr_columnx, rsr_columny, lsr_columnx, lsr_columny)
#     plt.xlabel(r'$x^i$')
#     plt.ylabel(r'$y^i$')
#     plt.show()


