#include <iostream>
#include <vector>
#include <limits>

using namespace std;

class Solution {
public:
    int maxProfit(vector<int>& prices) {
        int minprice = std::numeric_limits<int>::max(), maxprice = 0;
        for (auto& p : prices) {
            minprice = min(minprice, p);
            maxprice = max(maxprice, p - minprice);
        }
        return maxprice;
    }
};


int main(int argc, char** argv) {
    
    std::cout << "hello vslam" << std::endl;
    // vector<int> nums={1,2};
    vector<int> nums={7,1,5,3,6,4};
    int val = 2;
    Solution sol;
    int res = sol.maxProfit(nums);
    // for (auto& n : nums) {
    //     std::cout << n << " ";
    // }
    std::cout << res << std::endl;

    return 0;
}