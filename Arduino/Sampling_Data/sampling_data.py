import matplotlib.pyplot as plt
import csv

x = []
y = []

with open('flat.csv','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=';')
    for row in plots:
        y.append(float(row[0]))
x = [i for i in range(len(y))]
plt.plot(x,y, label='CSV data')
plt.xlabel('x')
plt.ylabel('y')
plt.title('Plot')
plt.legend()
plt.show()