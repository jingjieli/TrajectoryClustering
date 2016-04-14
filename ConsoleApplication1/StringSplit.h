#include <vector>
#include <string>

#ifndef STRINGSPLIT_H
#define STRINGSPLIT_H

std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems);
std::vector<std::string> split(const std::string &s, char delim);

#endif 