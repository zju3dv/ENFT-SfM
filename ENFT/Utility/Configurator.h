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

#ifndef _CONFIGURATOR_H_
#define _CONFIGURATOR_H_

#include <map>

class Configurator
{

public:

	inline std::string GetArgument(const std::string &directive, const std::string default_param = "") const
	{
		const DirectiveArgumentMap::const_iterator it = m_directiveArgumentMap.find(directive);
		if(it == m_directiveArgumentMap.end())
			return default_param;
		else {
			std::string tem = it->second;
#ifdef __LINUX__
			int n = tem.find_last_not_of("\r");
			if(n!=std::string::npos)
				tem.erase(n+1,tem.size()-n);
#endif
			return tem;
		}
	}
	inline int GetArgument(const std::string &directive, const int default_param) const
	{
		const DirectiveArgumentMap::const_iterator it = m_directiveArgumentMap.find(directive);
		if(it == m_directiveArgumentMap.end())
			return default_param;
		else
			return atoi(it->second.c_str());
	}
	inline std::vector<int> GetArgument(const std::string &directive, const std::vector<int> &default_param) const
	{
		const DirectiveArgumentMap::const_iterator it = m_directiveArgumentMap.find(directive);
		if(it == m_directiveArgumentMap.end())
			return default_param;
		else
		{
			char buf[MAX_LINE_LENGTH];
			strcpy(buf, it->second.c_str());
			const char *argumentStr = strtok(buf, ",");
			const int N = int(default_param.size());
			std::vector<int> arguments(N);
			for(int i = 0; i < N; ++i)
			{
				if(i == 0)
					sscanf(argumentStr, "(%d", &arguments[i]);
				else
					sscanf(argumentStr, "%d", &arguments[i]);
				argumentStr = strtok(NULL, ",");
			}
			return arguments;
		}
	}
	inline float GetArgument(const std::string &directive, const float default_param) const
	{
		const DirectiveArgumentMap::const_iterator it = m_directiveArgumentMap.find(directive);
		if(it == m_directiveArgumentMap.end())
			return default_param;
		else
			return float(atof(it->second.c_str()));
	}

	inline void SetArgument(const std::string &directive, const std::string &argument)
	{
		DirectiveArgumentMap::iterator it = m_directiveArgumentMap.find(directive);
		if(it == m_directiveArgumentMap.end())
			m_directiveArgumentMap.insert(DirectiveArgumentMap::value_type(directive, argument));
		else
			it->second = argument;
	}
	inline void SetArgument(const std::string &directive, const int &argument)
	{
		char buf[MAX_LINE_LENGTH];
		sprintf(buf, "%d", argument);
		DirectiveArgumentMap::iterator it = m_directiveArgumentMap.find(directive);
		if(it == m_directiveArgumentMap.end())
			m_directiveArgumentMap.insert(DirectiveArgumentMap::value_type(directive, buf));
		else
			it->second = buf;
	}
	inline void SetArgument(const std::string &directive, const std::vector<int> &argument)
	{
		std::string str;
		char buf[MAX_LINE_LENGTH];
		sprintf(buf, "(");
		const int N = int(argument.size());
		for(int i = 0; i < N; ++i)
		{
			if(i == 0)
				sprintf(buf + strlen(buf), "(%d", argument[i]);
			else
				sprintf(buf + strlen(buf), ", %d", argument[i]);
		}
	//	sprintf(buf + strlen(buf), "%d", argument);
		DirectiveArgumentMap::iterator it = m_directiveArgumentMap.find(directive);
		if(it == m_directiveArgumentMap.end())
			m_directiveArgumentMap.insert(DirectiveArgumentMap::value_type(directive, buf));
		else
			it->second = buf;
	}
	inline void SetArgument(const std::string &directive, const float &argument)
	{
		char buf[MAX_LINE_LENGTH];
		sprintf(buf, "%f", argument);
		DirectiveArgumentMap::iterator it = m_directiveArgumentMap.find(directive);
		if(it == m_directiveArgumentMap.end())
			m_directiveArgumentMap.insert(DirectiveArgumentMap::value_type(directive, buf));
		else
			it->second = buf;
	}

	inline bool Load(const char *fileName)
	{
		m_directiveArgumentMap.clear();
		FILE *fp = fopen(fileName, "r");
		if(!fp)
			return false;
		char buf[MAX_LINE_LENGTH];
		while(fgets(buf, MAX_LINE_LENGTH, fp))
		{
			int len = int(strlen(buf));
			int i, j;
			for(i = j = 0; i < len; i++)
			{
				if(buf[i] != 10 && buf[i] != ' ')
					buf[j++] = buf[i];
			}
			len = j;

			buf[len] = 0;
			if(len < 2 || buf[0] == '/' && buf[1] == '/')
				continue;
			const std::string directive = strtok(buf, "=");
			const char *argument = strtok(NULL, "=");
			if(!argument)
				continue;
			m_directiveArgumentMap.insert(DirectiveArgumentMap::value_type(directive, argument));
		}
		fclose(fp);
		printf("Loaded \'%s\'\n", fileName);
		return true;
	}

	void Print() const
	{
		printf("[Configurator]\n");
		for(DirectiveArgumentMap::const_iterator it = m_directiveArgumentMap.begin(); it != m_directiveArgumentMap.end(); it++)
			printf("  %s = %s\n", it->first.c_str(), it->second.c_str());
	}

private:

	typedef std::map<std::string, std::string> DirectiveArgumentMap;
	DirectiveArgumentMap m_directiveArgumentMap;

};

#endif