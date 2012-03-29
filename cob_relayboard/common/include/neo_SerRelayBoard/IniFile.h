/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Neobotix GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Neobotix nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#ifndef _Inifile_H
#define _Inifile_H

#include <iostream>
#include <vector>
#include <string>

//-------------------------------------------------------------------


/** 
 * Used to store persistend program configuration in INI-Files.
 * The INI-File is organized into sections ans Keys (variables) like
 * ordinary Windows INI-Files.
 * The write functions create a temporary file in the root directory.
 * If there is no write permission in this directory, they don't work.
 * @par sections: 
 * identifcator between '[' and ']', a section headline must not have blanks 
 * at the beginning neither right after or before the '[ ]'.
 * 	
 * @par sections contains keys:
 * identificator followed by '=' and the value, no blanks at the 
 * beginning of the line, nor after the '=' sign or between key 
 * and '=' sign. the  pBuf will contain the eventual blanks after
 * the '=' sign.
 * 
 * @par example:
 * @code
 * 	[section1]
 * 
 * 	key11=234
 * 	key12=yellow submarine
 * 	key13=13.5
 * 
 * 	[section2]
 * 
 * 	key21=
 * 	key22=13
 * @endcode
 *
 * Note: the write functions do not work in Linux
 * so they are disabled. If there is the need to
 * write data to IniFiles, a redesign of the whole
 * class with std::string functionality should be considered
 *
 * \ingroup UtilitiesModul
 */
 
class IniFile
{
public:

	/**
	 * Default constructor.
	 */
	IniFile();

	/**
	 * Constructor.
	 * @param fileName file name.
	 */
	IniFile(std::string fileName);
	~IniFile();

	/** 
	 * Sets file path of ini-file.
	 * Also verifies that file exists.
	 * @param fileName file name
	 * @param strIniFileUsedBy the name of the source file using the ini-file.
	 * This name will be printed if an error occurs.
	 * @param bCreate if true: create new file if the file does not exist (default: false)
	 * @return 0 if file exists or new file has been created sucesfully
	 */
	int SetFileName(std::string fileName, std::string strIniFileUsedBy = "", bool bCreate = false);


	/** 
	 * Write character string to INI-File.
	 * Like Windows Fn WriteProfileString().
	 * Comments in the same line as the variables will be deleted during write operations.
	 * @param pSect pointer to a null ended string containing the section title	without '[' and ']'
	 * @param pKey  pointer to a null ended string containing the key
	 * @param StrToWrite null ended string to write.
	 */	
	//int WriteKeyString(const char* pSect, const char* pKey, const std::string* pStrToWrite);

	/** 
	 * Write integer to INI-File.
	 * Like Windows Fn WriteProfileInt().
	 * Comments in the same line as the variables will be deleted during write operations.
	 * @param pSect pointer to a null ended string containing the section title without '[' and ']'
	 * @param pKey  pointer to a null ended string containing the key
	 * @param nValue integer to write.
	 */	
	//int WriteKeyInt(const char* pSect,const char* pKey,int nValue);

	/** 
	 * Write double to INI-File.
	 * Comments in the same line as the variables will be deleted during write operations.
	 * @param pSect pointer to a null ended string containing the section title without '[' and ']'
	 * @param pKey  pointer to a null ended string containing the key
	 * @param dValue double to write.
	 * @param StringLen total length of string into which the double will be converted
	 * @param decimals of double to store
	 */	
	//int WriteKeyDouble(const char* pSect,const char* pKey,double dValue,int StringLen=12,int decimals=5);

	/** 
	 * Read boolean from INI-File.
	 * The value writen will be either 'true' or 'false'.
	 * @param pSect pointer to a null ended string containing the section title without '[' and ']'
	 * @param pKey  pointer to a null ended string containing the key
	 * @param pValue pointer to boolean which will contain the value of the key.
	 * If the section or the key is not found, the value of *pValue remains unchanged
	 */	
	//int WriteKeyBool(const char* pSect, const char* pKey, bool bValue);

	/** 
	 * Read character string from INI-File.
	 * Like Windows Fn GetProfileString().
	 * @param pSect pointer to a null ended string containing the section title without '['	and ']'
	 * @param pKey  pointer to a null ended string containing the key
	 * @param pStrToRead will contain string read
	 * If the section or the key is not found, the value of *pStrToRead remains unchanged
	 */	
	int GetKeyString(const char* pSect,const char* pKey, std::string* pStrToRead, 
							bool bWarnIfNotfound = true);

	/** 
	 * Read integer from INI-File.
	 * Like Windows Fn GetProfileInt().
	 */	
	int GetKeyInt(const char* pSect,const char* pKey,int* pValue, 
							bool bWarnIfNotfound = true);

	/** 
	 * Read long from INI-File.
	 */	
	int GetKeyLong(const char* pSect,const char* pKey,long* pValue, 
							bool bWarnIfNotfound = true);

	/** 
	 * Read boolean from INI-File.
	 * The value can be either 'true' or 'false'.
	 * @param pSect pointer to a null ended string containing the section title without '['	and ']'
	 * @param pKey  pointer to a null ended string containing the key
	 * @param pValue pointer to boolean which will contain the value of the key.
	 * If the section or the key is not found, the value of *pValue remains unchanged
	 */	
	int GetKeyBool(const char* pSect, const char* pKey, bool* pValue, 
							bool bWarnIfNotfound = true);

	/** 
	 * Read double from INI-File.
	 * Current accuracy: 9 chars!!
	 * @param pSect pointer to a null ended string containing the section title without '['	and ']'
	 * @param pKey  pointer to a null ended string containing the key
	 * @param pValue pointer to double which will contain the value of the key.
	 * If the section or the key is not found, the vlaue of *pValue remains unchanged
	 */	
	int GetKeyDouble(const char* pSect,const char* pKey,double* pValue, 
							bool bWarnIfNotfound = true);

	/** 
	 * Read double from INI-File.
	 * Current accuracy: 9 chars!!
	 * @param pSect pointer to a null ended string containing the section title without '['	and ']'
	 * @param pKey  pointer to a null ended string containing the key
	 * @param pValue pointer to double which will contain the value of the key.
	 * If the section or the key is not found, the vlaue of *pValue remains unchanged
	 */	
	int GetKeyDouble(const char* pSect,const char* pKey,double* pValue, double dDefault,
							bool bWarnIfNotfound = true);

	/** 
	 * Read character string from INI-File.
	 * Like Windows Fn GetProfileString().
	 * @param pSect pointer to a null ended string containing the section title without '['	and ']'
	 * @param pKey  pointer to a null ended string containing the key
	 * @param StrToRead will contain string read
	 * If the section or the key is not found, the value of *pStrToRead remains unchanged
	 */	
	int GetKey(const char* pSect,const char* pKey, std::string* pStrToRead, bool bWarnIfNotfound = true);

	/** 
	 * Read integer from INI-File.
	 * Like Windows Fn GetProfileInt().
	 */	
	int GetKey(const char* pSect,const char* pKey,int* pValue, bool bWarnIfNotfound = true);

	/** 
	 * Read boolean from INI-File.
	 * The value can be either 'true' or 'false'.
	 * @param pSect pointer to a null ended string containing the section title without '['	and ']'
	 * @param pKey  pointer to a null ended string containing the key
	 * @param pValue pointer to boolean which will contain the value of the key.
	 * If the section or the key is not found, the value of *pValue remains unchanged
	 */	
	int GetKey(const char* pSect, const char* pKey, bool* pValue, bool bWarnIfNotfound = true);

	/** 
	 * Read double from INI-File.
	 * Current accuracy: 9 chars!!
	 * @param pSect pointer to a null ended string containing the section title without '['	and ']'
	 * @param pKey  pointer to a null ended string containing the key
	 * @param pValue pointer to double which will contain the value of the key.
	 * If the section or the key is not found, the vlaue of *pValue remains unchanged
	 */	
	int GetKey(const char* pSect,const char* pKey,double* pValue, bool bWarnIfNotfound = true);

private:
	
	/**
	 *
	 */
	int FindSection(const char* sect, bool bWarnIfNotfound = true);
	
	/**
	 *
	 */
	int FindKey(const char* sSection, const char* sKey, bool bWarnIfNotfound = true);
	
	/**
	 *
	 */
	int FindNextLine(std::vector<char>& NewLine, int& CharInd);

	/**
	 * Write character string to INI-File.
	 * Like Windows Fn WriteProfileString().
	 * @param pSect pointer to a null ended string containing the section title without '['	and ']'
	 * @param pKey  pointer to a null ended string containing the key
	 * @param pBuf null ended string to write.
	 */	
	int WriteKeyValue(const char* pSect,const char* pKey,const char* pBuf);

	/**
	 * Read character string from INI-File.
	 * Like Windows Fn GetProfileString().
	 * @param pSect pointer to a null ended string containing the section title without '['	and ']'
	 * @param pKey  pointer to a null ended string containing the key
	 * @param pBuf pointer to a character buffer of length lenBuf  string which will 
	 * contain the value of the key.
	 * If the section or the key is not found, the vlaue of *pBuf remains unchanged
	 * @param lenBuf the maximal length of szBuf (including terminating \0).
	 * @return	the length of the string read or RF_E if error
	 */	
	int GetKeyValue(const char* pSect,const char* pKey, char* pBuf, int lenBuf, 
							bool bWarnIfNotfound = true);

	/**
	 * Skips chars in line until Endchar.
	 * return:	- Nr of chars skipped if successful
	 * 			- RF_E if end of line (\n) or end of file before Endchar is found
	 */
	int SkipLineUntil(FILE* pFile, const char EndChar);

	/** 
	 * Reads chars in line until Endchar into string.
	 * return:	- Nr of chars read if successful
	 * 			- RF_E if end of line (\n) or end of file before Endchar is found.
	 * In this case, the string will contain the chars read so far.
	 */
	int ReadLineUntil(FILE* pFile, const char EndChar, std::string& ReadIntoStr);

	bool m_bFileOK;

	/**
	 * Vector to store the chars of the most recently read line.
	 */
	std::vector<char> m_CurLine;

	/**
	 * Size of the vector CurLine.
	 */
	const int m_vectorSize;

	/**
	 * Index of the current character in the current line.
	 */
    int m_CurCharInd;

	std::string m_fileName;
	std::string m_strIniFileUsedBy;	//used for debug to inidcate user class of ini-file
	FILE* f;
};

#endif

