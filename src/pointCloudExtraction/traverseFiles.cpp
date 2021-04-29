#include <iostream>  
#include <vector>
#include <algorithm>
#include <stdlib.h>  
#include <stdio.h>  
#include <string.h>  
#include <direct.h>  
#include <io.h>  
#include "traverseFiles.h"
using namespace std;

vector<string> getFiles(string cate_dir)
{
    vector<string> files;
    //char current_address[100];
    //memset(current_address, 0, 100);
    //getcwd(current_address, 100); //获取当前路径  
    //cout << current_address << endl;
    //strcat(current_address, "\\*");
    //cout << current_address << endl;
    _finddata_t file;
    intptr_t lf;
    //输入文件夹路径  
    if ((lf = _findfirst(cate_dir.c_str(), &file)) == -1) {
        cout << cate_dir << " not found!!!" << endl;
    }
    else {
        while (_findnext(lf, &file) == 0) {
            //输出文件名  
            //cout<<file.name<<endl;  
            if (strcmp(file.name, ".") == 0 || strcmp(file.name, "..") == 0 || file.attrib & _A_SUBDIR)
                continue;
            string fn = file.name;
            string suffix_str = fn.substr(fn.find_last_of('.') + 1);
            if(strcmp(suffix_str.c_str(), "jpg") == 0 || strcmp(suffix_str.c_str(), "raw") == 0)
                files.push_back(fn);
        }
    }
    _findclose(lf); 

    //排序，按从小到大排序  
    sort(files.begin(), files.end());
    return files;
}
