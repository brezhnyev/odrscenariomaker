#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

using namespace std;

int main(int argc, char ** argv)
{
    if (argc < 2)
    {
        cout << "Syntax: " << argv[0] << " path/to/file.txt" << endl;
        return 0;
    }

    ifstream ifs(argv[1], ios::binary);

    stringstream ss;
    ss << ifs.rdbuf();
    string txt = ss.str();
    vector<char> osi; osi.reserve(txt.size());
    string delim = "$$__$$";

    union int2bytes
    {
        int i;
        unsigned char b[4];
    } i2b;

    for (int i = 0, j = 0; i < txt.size();)
    {
        if (i+delim.size() <= txt.size() && txt.substr(i,delim.size()) == delim)
        {
            i2b.i = i-j;
            osi.push_back(i2b.b[0]);
            osi.push_back(i2b.b[1]);
            osi.push_back(i2b.b[2]);
            osi.push_back(i2b.b[3]);
            while (j!=i) osi.push_back(txt[j++]);
            i+=delim.size();
            j=i; 
        } 
        else ++i;
    }

    ifs.close();

    ofstream ofs((string(argv[1]).substr(0, string(argv[1]).size()-3) + "osi").c_str(), ios::binary);

    ofs.write(&osi[0], osi.size());
    ofs.close();

    return 0;

}