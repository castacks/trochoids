## MO
import os
import pandas as pd
import matplotlib.pyplot as plt

### Set your path to the folder containing the .csv files
PATH = 'csv_files/' # Use your path

fileNames = os.listdir(PATH)
fileNames = [file for file in fileNames if '.csv' in file]
fileNames.sort()
dataframes_list = []
files = []
### Loop over all files
for file in fileNames:
    df = pd.read_csv(PATH + file, sep=',', header=None)
    dataframes_list.append(df)
    files.append(file)
    
for dataframe, file in zip(dataframes_list, files):    
    file = file.split('.')[0]
    file = file.replace('_', ' ')
    file = file.capitalize()

    plt.scatter(dataframe[0], dataframe[1], label=file)
    plt.axis('equal')

    plt.legend(loc = 'upper left')
    plt.title(file)
    plt.grid()
    plt.savefig('figures/'+ file + '.png', transparent=True)
    plt.show()
    