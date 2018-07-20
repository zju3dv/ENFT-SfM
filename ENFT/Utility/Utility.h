////////////////////////////////////////////////////////////////////////////
//  Copyright 2017-2018 Computer Vision Group of State Key Lab at CAD&CG, 
//  Zhejiang University. All Rights Reserved.
//
//  For more information see <https://github.com/ZJUCVG/ENFT-SfM>
//  If you use this code, please cite the corresponding publications as 
//  listed on the above website.
//
//  Permission to use, copy, modify and distribute this software and its
//  documentation for educational, research and non-profit purposes only.
//  Any modification based on this work must be open source and prohibited
//  for commercial use.
//  You must retain, in the source form of any derivative works that you 
//  distribute, all copyright, patent, trademark, and attribution notices 
//  from the source form of this work.
//   
//
////////////////////////////////////////////////////////////////////////////

#ifndef _UTILITY_H_
#define _UTILITY_H_
#ifdef __LINUX__
#include <stdarg.h>
#endif
namespace IO {

inline void Assert(const bool expression, char *format, ...) {
    //assert(expression);
    if(expression)
        return;

    va_list args;
    char buf[MAX_LINE_LENGTH];
    va_start(args, format);
    vsprintf(buf, format, args);
    va_end(args);
    printf("%s", buf);
    exit(0);
}

template<typename TYPE>
inline void AssertEqual(const TYPE &v1, const TYPE &v2) {
    if(EQUAL(v1, v2))
        return;
    printf("%f - %f = %f\n", v1, v2, v1 - v2);
    exit(0);
}

inline void StringSaveB(const std::string &str, char *buf, FILE *fp) {
    sprintf(buf, "%s\n", str.c_str());
    fwrite(buf, 1, strlen(buf), fp);
}

inline void StringLoadB(std::string &str, char *buf, FILE *fp) {
    fgets(buf, MAX_LINE_LENGTH, fp);
    const int len = (int)(strlen(buf));
    if(buf[len - 1] == 10)
        buf[len - 1] = 0;
    str = buf;
}

template<class TYPE>
inline void VectorSaveB(const std::vector<TYPE> &vec, FILE *fp) {
    const uint size = uint(vec.size());
    fwrite(&size, sizeof(uint), 1, fp);
    if(size != 0)
        fwrite(vec.data(), sizeof(TYPE), size, fp);
}

template<class TYPE>
inline uint VectorLoadB(std::vector<TYPE> &vec, FILE *fp) {
    uint size;
    fread(&size, sizeof(uint), 1, fp);
    vec.resize(size);
    if(size != 0)
        fread(vec.data(), sizeof(TYPE), size, fp);
    return size;
}

template<class TYPE>
inline void VectorSetSaveB(const std::vector<std::vector<TYPE> > &vecs,
                           FILE *fp) {
    const uint size = uint(vecs.size());
    fwrite(&size, sizeof(uint), 1, fp);
    for(uint i = 0; i < size; ++i)
        VectorSaveB(vecs[i], fp);
}

template<class TYPE>
inline void VectorSetLoadB(std::vector<std::vector<TYPE> > &vecs, FILE *fp) {
    uint size;
    fread(&size, sizeof(uint), 1, fp);
    vecs.resize(size);
    for(uint i = 0; i < size; ++i)
        VectorLoadB(vecs[i], fp);
}

inline void SaveValues(const char *fileName, const std::vector<float> &vals) {
    
    FILE *fp = fopen( fileName, "w");
    const int N = int(vals.size());
    for(int i = 0; i < N; ++i)
        fprintf(fp, "%f\n", vals[i]);
    fclose(fp);
}
inline void SaveHistogram(const char *fileName, const std::vector<float> &vals,
                          const uint nBins) {
    int i;
    float val, valMin = FLT_MAX, valMax = -FLT_MAX;
    const int N = int(vals.size());
    for(i = 0; i < N; ++i) {
        val = vals[i];
        if(val < valMin)
            valMin = val;
        if(val > valMax)
            valMax = val;
    }
    const float binWidth = (valMax - valMin) / (nBins - 1);

    uint iBin;
    std::vector<uint> hist(N, 0);
    for(i = 0; i < N; ++i) {
        iBin = uint((vals[i] - valMin) / binWidth);
        ++hist[iBin];
    }

    
    FILE *fp = fopen( fileName, "w");
    for(iBin = 0, val = valMin; iBin < nBins; ++iBin, val += binWidth)
        fprintf(fp, "%f %d\n", val, hist[iBin]);
    fclose(fp);
}

inline std::string ExtractFileDirectory(const std::string &fileName) {
    const std::string::size_type i1 = fileName.rfind('/'),
                                 i2 = fileName.rfind('\\');
    if(i1 == std::string::npos && i2 == std::string::npos)
        return std::string();
    else if(i1 != std::string::npos && i2 == std::string::npos)
        return fileName.substr(0, i1 + 1);
    else if(i1 == std::string::npos && i2 != std::string::npos)
        return fileName.substr(0, i2 + 1);
    else if(i1 > i2)
        return fileName.substr(0, i1 + 1);
    else
        return fileName.substr(0, i2+1);
}

inline std::string RemoveFileDirectory(const std::string &fileName) {
    const std::string::size_type i1 = fileName.rfind('/'),
                                 i2 = fileName.rfind('\\');
    if(i1 == std::string::npos && i2 == std::string::npos)
        return fileName;
    else if(i1 != std::string::npos && i2 == std::string::npos)
        return fileName.substr(i1 + 1, fileName.size());
    else if(i1 == std::string::npos && i2 != std::string::npos)
        return fileName.substr(i2 + 1, fileName.size());
    else if(i1 > i2)
        return fileName.substr(i1 + 1, fileName.size());
    else
        return fileName.substr(i2+1, fileName.size());
}

inline std::string ExtractFileExtension(const std::string &fileName) {
    const std::string::size_type i = fileName.rfind('.');
    if(i == std::string::npos)
        return std::string();
    else
        return fileName.substr(i + 1, fileName.size());
}

inline std::string RemoveFileExtension(const std::string &fileName) {
    const std::string::size_type i = fileName.rfind('.');
    if(i == std::string::npos)
        return fileName;
    else
        return fileName.substr(0, i);
}

inline std::string RemoveFileDirectoryAndExtension(const std::string
        &fileName) {
    return RemoveFileDirectory(RemoveFileExtension(fileName));
}

inline std::string RemovePrefix(const std::string &fileName,
                                const std::string &prefix) {
    if(fileName == "" || fileName.find(prefix) != 0)
        return fileName;
    else
        return fileName.substr(prefix.length(), fileName.length());
}

inline std::string AppendSuffix(const std::string &fileName,
                                const std::string &suffix) {
    return RemoveFileExtension(fileName) + suffix + "." + ExtractFileExtension(
               fileName);
}

inline std::string ReplaceFileDirectory(const std::string &fileName,
                                        const std::string &dirSrc, const std::string &dirDst) {
    if(fileName == "" || fileName.find(dirSrc) != 0)
        return fileName;
    else
        return dirDst + fileName.substr(dirSrc.length(), fileName.length());
}

inline std::string ReplaceSubString(const std::string &str,
                                    const std::string &strSrc, const std::string &strDst) {
    std::string::size_type pos;
    std::string res = str;
    while(1) {
        if((pos = res.find(strSrc)) == std::string::npos)
            break;
        res.replace(pos, strSrc.length(), strDst);
    }
    return res;
}

inline std::string InsertSuffix(const std::string &fileName,
                                const std::string &suffix) {
    const std::string::size_type i = fileName.rfind('.');
    if(i == std::string::npos)
        return fileName + suffix;
    else
        return fileName.substr(0, i) + suffix + fileName.substr(i, fileName.length());
}

inline int ExtractFileNumber(const std::string &fileName) {
    int i2 = int(fileName.length());
    while(--i2 >= 0 && !isdigit(fileName[i2]));
    int i1 = ++i2;
    while(--i1 >= 0 && isdigit(fileName[i1]));
    if(++i1 == i2)
        return -1;
    else
        return atoi(fileName.substr(i1, i2 - i1).c_str());
}

inline std::string IncreaseFileNumber(const std::string &fileName,
                                      const int &incr) {
    const int len = int(fileName.length());
    int i2 = len;
    while(--i2 >= 0 && !isdigit(fileName[i2]));
    int i1 = ++i2;
    while(--i1 >= 0 && isdigit(fileName[i1]));
    const int number = ++i1 == i2 ? incr : atoi(fileName.substr(i1,
                       i2 - i1).c_str()) + incr;
    const int width1 = i2 - i1, width2 = int(log10f(float(number)));
    const int width = width1 > width2 ? width1 : width2;

    char buf[MAX_LINE_LENGTH];
    switch(width) {
        case 2:
            sprintf(buf, "%.2d", number);
            break;
        case 3:
            sprintf(buf, "%.3d", number);
            break;
        case 4:
            sprintf(buf, "%.4d", number);
            break;
        case 5:
            sprintf(buf, "%.5d", number);
            break;
        case 6:
            sprintf(buf, "%.6d", number);
            break;
        case 7:
            sprintf(buf, "%.7d", number);
            break;
        case 8:
            sprintf(buf, "%.8d", number);
            break;
        case 9:
            sprintf(buf, "%.9d", number);
            break;
        case 10:
            sprintf(buf, "%.10d", number);
            break;
        default:
            sprintf(buf, "%d", number);
            break;
    }
    return fileName.substr(0, i1) + buf + fileName.substr(i2, len - i2);
}
}

#endif