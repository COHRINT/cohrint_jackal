#!/usr/bin/python3

import sys

import itertools

localinos = None

def get_combos(size_d):
    l = itertools.combinations(list(range(1,size_d + 1)), 2)
    combos = []
    for i in l:
        combos.append(i)
        combos.append(( i[1], i[0] ))
    return combos

def increment_localinos(index):
    new_index = ( index + 1) % len(localinos)
    return localinos[new_index]




def increment_tag_anchor(tag, anchor,d):
    if tag > d or anchor > d or tag == anchor:
        print("bad input")
        return 1,2
    # first check to increment the tag
    anchor = (anchor + 1) % ( d + 1)
    if anchor == 0:
        anchor = 1
        tag += 1
    if anchor == tag:
        if not (anchor % d):
            tag = 1
            anchor = 2
        else:
            anchor += 1
        
    return tag, anchor

if __name__ == "__main__":
    # tag = int(sys.argv[1])
    # anchor = int(sys.argv[2])
    # size = int(sys.argv[3])
    localinos = get_combos(int(sys.argv[2]))
    # make the index zero when we add a new one    
    print(localinos)
    print(increment_localinos(int(sys.argv[1])))
