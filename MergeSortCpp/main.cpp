#include <iostream>
#include <vector>
#include "mergeSort.h"

using namespace Wayne;
using namespace std;

int main()
{
    vector<int> data = {1,9,8,6,5,9,0,2,4,10,13,12,8}; 

    unique_ptr<vector<int>> dataPtr = make_unique<vector<int>>(move(data));

    MergeSort ms(move(dataPtr));

    dataPtr = ms.sort();

    for(auto a: *dataPtr)
        cout<<"data = "<<a<<endl;

}