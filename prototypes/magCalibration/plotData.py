import pandas as pd
import matplotlib.pyplot as plt
import csv


df = pd.read_csv('output.csv')

df.plot(x='x', y='y', kind='scatter')
plt.show()