def meanSquareError(x, y):
    summation = 0  #variable to store the summation of differences
    n = len(x) #finding total number of items in list
    for i in range (0,n):  #looping through each element of the list
        difference = x[i] - y[i]  #finding the difference between observed and predicted value
        squared_difference = difference**2  #taking square of the differene 
        summation = summation + squared_difference  #taking a sum of all the differences
        MSE = summation/n  #dividing summation by total values to obtain average
    return MSE