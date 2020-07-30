#Functions for processing arrays

#check if any data in this array slice is less than the target value
def check_slice_less(slice, dis):
    slice = repnan(slice)
    if(average(slice) < dis):
            return True
    return False

#check if any data in this array slice is more than the target value
def check_slice_more(slice, dis):
    slice = repnan(slice)
    if(average(slice) > dis):
        return True
    return False

def average(lst): 
    if len(lst) == 0:
        return 0
    return sum(lst) / len(lst) 

#cehck to see if all values in the slice are the same
def check_slice_same(slice):
    slice = nonan(slice)
    x = slice[0]
    for i in slice:
        if(i != x):
            return False
    return True

#remove all of the nan values from the slice
def nonan(slice):
    filtered = []
    for x in slice:
        if(str(x) != "nan"):
            filtered.append(x)
    return filtered

#replace all of the nan values from the slice
def repnan(slice):
    filtered = []
    for x in slice:
        if(str(x) != "nan"):
            filtered.append(x)
        else:
            filtered.append(0.4)
    return filtered

#check to see if the entire slice is nan values
def checknan(slice):
    filtered = []
    for x in slice:
        if(str(x) != "nan"):
            filtered.append(x)

    if not filtered:
        return True
    else:
        return False

#get the direction based off of position in the navigation array
def direction(x):
    if x==0:
        return 0
    elif x==1:
        return -90
    elif x==2:
        return 180
    elif x==3:
        return 90