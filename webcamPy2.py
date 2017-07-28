import webcamPy

arr=[]
arr.append(0)
arr.append(0)

while arr[0]==0 and arr[1]==0:
	arr=webcamPy.findCenter()

print arr[0]
print arr[1]