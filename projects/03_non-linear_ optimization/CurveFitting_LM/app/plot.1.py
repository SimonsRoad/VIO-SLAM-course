import matplotlib.pyplot as plt

filename = "../build/output.txt" 

X1,X2,Y = [],[],[]
with open(filename, 'r') as f:
    lines = f.readlines()
    for line in lines:
        value = [float(s) for s in line.split()]
        X1.append(value[0])
        X2.append(value[1])
        Y.append(value[2])
print(X1)        
print(X2)
print(Y)

l1 = plt.plot(Y, X1, color='blue',marker='o',label='u')
# l2 = plt.plot(Y, X2, color='green',linewidth=1.0,linestyle='--',marker='s',label='F(x)')
plt.legend()

# plt.grid()
plt.xlim((-10,220))
plt.ylim((-100,6000))
plt.show()