#pragma once

// A simpler folder reader
#include <algorithm>
#ifdef _WIN32
#include "windows/dirent.h"
#else
#include <dirent.h>
#endif
#include <set>

#include <iostream>

inline bool hasEnding (std::string const &fullString, std::string const &ending)
{
    if (fullString.length() >= ending.length()) 
    {
        return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
    } 
    else 
    {
        return false;
    }
}

inline bool hasEndingLower (std::string const &fullString_, std::string const &_ending)
{
    std::string fullstring = fullString_, ending = _ending;
    std::transform(fullString_.begin(),fullString_.end(),fullstring.begin(),::tolower); // to lower
    return hasEnding(fullstring,ending);
}

// compares strings like 100 and 12 and returns 12 < 100 (natural comparison)
struct NaturalCompare 
{
    bool operator() (const std::string & s1, const std::string & s2) const
    {
        if (s1.size() != s2.size())
        {
            return s1.size() < s2.size();
        }
        return s1 < s2;
    }
};

// same as less<string>
// compares strings like 100 and 12 and returns 100 < 12 (lexicografic comparison)
struct LexCompare 
{
    bool operator() (const std::string & s1, const std::string & s2) const
    {
        return s1 < s2;
    }
};

// ex. 
// FolderReader<NaturalCompare> ncfr; (natural)
// FolderReader<LexCompare> lcfr; (lexicografic)
// FolderReader<> dfr; // default (which is lexicograpic)

template<typename Container = std::set<std::string, NaturalCompare> >
class FolderReader
{
public:
    FolderReader(std::string dir_name, int start = 0) : m_dir_name(dir_name), m_filesCounter(start)
    {
        open(dir_name);
    }
    FolderReader() : m_dir_name(""), m_filesCounter(0) {}
    
public:
    void open(std::string dir_name)
    {
        using namespace std;
        
        m_dir_name = dir_name;
        if (dir_name.empty()) return;
        
        //open a directory the POSIX way
        
        DIR *dp;
        struct dirent *ep;
        dp = opendir (dir_name.c_str());
        
        if (dp != NULL)
        {
            while (ep = readdir (dp)) 
            {
                if (ep->d_name[0] != '.')
                {
                    m_files.insert(ep->d_name);
                }
            }
            (void) closedir (dp);
        }
        else 
        {
            cerr << ("Couldn't open the directory\n");
        }
        // open may be called to keep up to a changing folder (ex. new files are being written). Call rewind is necessary:
        rewind();
    }
    
    std::string getNext(std::string extension) 
    {
        using namespace std;
        
        if (m_filesCounter >= m_files.size()) return "";
        
        typename Container::iterator it = m_files.begin(); advance(it, m_filesCounter);
        
        while (m_filesCounter < m_files.size() && it != m_files.end())
        { 
            string name = *it;
            if (name[0] == '.' || !(hasEndingLower(name,extension)))
            {
                ++m_filesCounter; ++it;
                continue;
            }
            else
            {
                ++m_filesCounter;
                return string(m_dir_name).append("/").append(name);
            }
        }
        
        if (m_filesCounter >= m_files.size()) return "";
		
		return "";
    }

    void rewind()
    {
        m_filesCounter = 0;
    }
    
    Container m_files;
    std::string m_dir_name;
    int m_filesCounter;
};
