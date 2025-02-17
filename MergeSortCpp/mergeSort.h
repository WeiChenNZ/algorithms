#pragma once

#include<vector>
#include<memory>

namespace Wayne{

    class MergeSort{
        public:
            MergeSort(std::unique_ptr<std::vector<int>> dataPtr_):dataPtr(move(dataPtr_)){}
            std::unique_ptr<std::vector<int>> sort();

        private:
            std::unique_ptr<std::vector<int>> dataPtr; 

            void merge(int, int);
            void mergeAndSort(int, int);
    };

}