#include "mergeSort.h"

using namespace std;
using namespace Wayne;


    unique_ptr<std::vector<int>> MergeSort::sort()
    {   
        mergeAndSort(0, dataPtr->size()-1);
        return move(dataPtr);
    }

    //recursive method
    void MergeSort::mergeAndSort(int left, int right)
    {
        int mid = left + (right - left)/2;
        
        if(left < right)
        {
            mergeAndSort(left, mid);
            mergeAndSort(mid+1, right);
            merge(left, right);
        }
    }

    void MergeSort::merge(int left, int right)
    {
        int mid = left + (right - left)/2;

        vector<int> leftVec, rightVec;

        for(int i = left; i < mid+1; i++)
        {
            leftVec.push_back(dataPtr->at(i));
        }
        for(int i = mid+1; i < right+1; i++)
        {
            rightVec.push_back(dataPtr->at(i));
        }

        //merge to sub vectors
        int i = 0;
        while(leftVec.size() != 0 || rightVec.size() != 0)
        {
            if(leftVec.begin() != leftVec.end() && rightVec.begin() != rightVec.end())
            {
                if(*leftVec.begin() < *rightVec.begin())
                {
                    dataPtr->at(left+i) = *leftVec.begin();
                    leftVec.erase(leftVec.begin());
                }
                else
                {
                    dataPtr->at(left+i) = *rightVec.begin();
                    rightVec.erase(rightVec.begin());
                }
            }
            else if(leftVec.size() == 0 && rightVec.size() != 0)
            {
                dataPtr->at(left + i) = *rightVec.begin();
                rightVec.erase(rightVec.begin());
            }
            else if(rightVec.size() == 0 && leftVec.size() != 0)
            {
                dataPtr->at(left + i) = *leftVec.begin();
                leftVec.erase(leftVec.begin());
            }
            i++;
        }

    }




