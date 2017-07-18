/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 

#ifndef _Inifile_H
#define _Inifile_H

#include <iostream>
#include <vector>
#include <string>
#include <stdio.h>

//-------------------------------------------------------------------


/**
 * Used to store persistend program configuration in INI-Files.
 * The INI-File is organized into sections and Keys (variables) like
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
	 * @param bWarnIfNotfound print a warning message if the section is not found and therefore is created newly.
	 */
	int WriteKeyString(const char* pSect, const char* pKey, const std::string* pStrToWrite, bool bWarnIfNotfound = true);

	/**
	 * Write integer to INI-File.
	 * Like Windows Fn WriteProfileInt().
	 * Comments in the same line as the variables will be deleted during write operations.
	 * @param pSect pointer to a null ended string containing the section title without '[' and ']'
	 * @param pKey  pointer to a null ended string containing the key
	 * @param nValue integer to write.
	 * @param bWarnIfNotfound print a warning message if the section is not found and therefore is created newly.
	 */
	int WriteKeyInt(const char* pSect,const char* pKey,int nValue, bool bWarnIfNotfound = true);

	/**
	 * Write double to INI-File.
	 * Comments in the same line as the variables will be deleted during write operations.
	 * @param pSect pointer to a null ended string containing the section title without '[' and ']'
	 * @param pKey  pointer to a null ended string containing the key
	 * @param dValue double to write.
	 * @param StringLen total length of string into which the double will be converted
	 * @param decimals of double to store
	 * @param bWarnIfNotfound print a warning message if the section is not found and therefore is created newly.
	 */
	int WriteKeyDouble(const char* pSect,const char* pKey,double dValue,int StringLen=12,int decimals=5, bool bWarnIfNotfound = true);

	/**
	 * Read boolean from INI-File.
	 * The value writen will be either 'true' or 'false'.
	 * @param pSect pointer to a null ended string containing the section title without '[' and ']'
	 * @param pKey  pointer to a null ended string containing the key
	 * @param pValue pointer to boolean which will contain the value of the key.
	 * If the section or the key is not found, the value of *pValue remains unchanged
	 * @param bWarnIfNotfound print a warning message if the section is not found and therefore is created newly.
	 */
	int WriteKeyBool(const char* pSect, const char* pKey, bool bValue, bool bWarnIfNotfound = true);

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

	/**
	 * Find the section name after the given section.
	 * If prevSect is NULL, get the first section name.
	 * @param pSect pointer to a null ended string which will contain the section title without '[' and ']'.
	 * @param prevSect pointer to a null ended string contraing the previous section title without '[' and ']'.
	 * @param bWarnIfNotfound print a warning message if the section is not found.
	 * If the section is not found, the value of *sect is not defined.
	 */
	int FindNextSection(std::string* pSect, std::string prevSect, bool bWarnIfNotfound = true);

private:

	int FindSection(const char* sect, bool bWarnIfNotfound = true);
	int FindKey(const char* skey,	bool bWarnIfNotfound = true);
	int FindNextLine(std::vector<char>& NewLine, int& CharInd);

	/**
	 * Write character string to INI-File.
	 * Like Windows Fn WriteProfileString().
	 * @param pSect pointer to a null ended string containing the section title without '['	and ']'
	 * @param pKey  pointer to a null ended string containing the key
	 * @param pBuf null ended string to write.
	 * @param bWarnIfNotfound print a warning message if the section is not found and therefore is created newly.
	 */
	int WriteKeyValue(const char* pSect,const char* pKey,const char* pBuf, bool bWarnIfNotfound = true);

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

