#include <iostream>     // cout
#include <sstream>      // istringstream
#include <string>       // string
#include <fstream>      // file operation

#define stringify(Variable) (#Variable)

using namespace std;


struct Tes{
    string file;
    string url;
}tes;

int loadConfig(std::string filename)
{
    std::ifstream file (filename);
    std::stringstream buffer;

    if (file)
    {
        //copy to buffer
        buffer << file.rdbuf();
        file.close();
    }
    //file not found
    else
    {
        return 0;
    }
    
    std::string line;
    while( std::getline(buffer, line) )
    {
        std::istringstream is_line(line);
        if (line[0] == '#')
        {
            //comment line
        } 
        else
        {
            std::string key;
            if( std::getline(is_line, key, '=') )
            {
                std::string value;
                if( std::getline(is_line, value) ) 
                {
                    if ((key.compare(stringify(tes.file)))==0)
                    {
                        //assign
                        //variable name (based on key)
                        tes.file = value;
                    }
                    /*
                    else
                    if (key.compare(..)==0)
                    {

                    }
                    */
                }
            }
        }   
    }
    return 1;
}

int main()
{
    if (!loadConfig("tesconfig.cfg"))
        std::cout << "error loading config file. file not found" << std::endl;
    
    std::cout << tes.file <<std::endl;
    std::cout << tes.url <<std::endl;
    
    
    char c;
    std::cin >> c;
}

