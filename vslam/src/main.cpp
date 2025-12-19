#include <iostream>
#include <vector>

using namespace std;

class Solution {
public:
    int removeDuplicates(vector<int>& nums) {
        int num = nums.size();
        int slow=2, fast=2;
        while(fast < num) {
            if (nums[fast] != nums[slow-2]) {
                nums[slow] = nums[fast];
                slow++;
            }
            fast++;
        }
        return slow;
    }
};


int main(int argc, char** argv) {
    
    std::cout << "hello vslam" << std::endl;
    vector<int> nums={2,2,1,1,1,2,2};
    int val = 2;
    Solution sol;
    sol.removeDuplicates(nums);
    for (auto& n : nums) {
        std::cout << n << " ";
    }
    std::cout << std::endl;

    return 0;
}