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


#include <neo_SerRelayBoard/IniFile.h>
#include <stdlib.h>
#include <string.h>

using namespace std;

//-------------------------------------------------------------------
IniFile::IniFile() :
	m_vectorSize(500),
	m_CurCharInd(0)
{
	m_bFileOK=false;
	m_CurLine.resize(m_vectorSize);
}

//--------------------------------------------------------------------------------

IniFile::IniFile(std::string fileName) :
	m_vectorSize(500),
	m_CurCharInd(0)
{
	m_bFileOK=false;
	m_CurLine.resize(m_vectorSize);
	
	if(fileName != "")
	{ 
		SetFileName(fileName);
	}
}

//--------------------------------------------------------------------------------
IniFile::~IniFile()
{
}

//--------------------------------------------------------------------------------
int IniFile::SetFileName(std::string fileName, std::string strIniFileUsedBy, bool bCreate)
{
	m_fileName = fileName;
	m_strIniFileUsedBy = strIniFileUsedBy;

	if ((f = fopen(m_fileName.c_str(),"r")) == NULL)
	{	
		if (bCreate == true)
		{
			f = fopen(m_fileName.c_str(),"w");	// create new file
			LOGINFO("creating new INI-File " << m_fileName.c_str() << endl);
			fclose(f);
		}
		else
		{
			std::cout <<"INI-File not found " << m_fileName.c_str() << endl;
			char c;
			std::cin >> c;
			exit(-1);
		}
	}
	else
	{
		fclose(f);
	}
	
	m_bFileOK = true;
	
	return 0;
}

/*

//--------------------------------------------------------------------------------
int IniFile::WriteKeyString(const char* pSect, const char* pKey, const std::string* pStrToWrite)
{	
	string StrWithDelimeters = '"' + *pStrToWrite + '"'; 
	
	return WriteKeyValue(pSect, pKey, StrWithDelimeters.c_str());
}

//--------------------------------------------------------------------------------
int IniFile::WriteKeyValue(const char* szSect,const char* szKey,const char* szValue)
{
	if (!m_bFileOK)
	{
		return -1;
	}
	
	FILE* ftemp;
	int   lS,lK,i,bEoff;
	int   bFoundSect,bFoundKey;
	char  c;
    long fpos;

	// verifications
	bFoundSect = 1;
	bFoundKey = 0;
	bEoff = 0;
	lS = strlen(szSect);
	lK = strlen(szKey);
	if ((lS * lK) == 0)
	{
		return -1;
	}

	// open file
	f = fopen(m_fileName.c_str(),"r");
	if (f == NULL)
	{
		LOGERROR("INI-File not found " << m_fileName.c_str() << endl);
		return -1;
	}
	if ((ftemp = tmpfile()) == NULL)	// nicht unter UNIX (anderes include?)
	{
		// PrintErr(errno);
		return -1;
	}


	// search section and key
	if (FindSection(szSect))
	{	
		// fprintf(f,"[%s]\n\n",szSect);
		bFoundSect = 0;
	}
	
	fpos = ftell(f);
	
	if (bFoundSect)
	{
		if (!FindKey(szKey))
		{
			bFoundKey = 1;
		}
		
		fpos = ftell(f);
	}
	
	if (feof(f))
	{
		bEoff = 1;
	}

	// updating the file
	fseek(f, 0, SEEK_SET);
	for (i = 0; i < fpos; i++)
	{
		fscanf(f,"%c",&c);
		fprintf(ftemp,"%c",c);
		if (c == '\n')
		{
			i++;
		}
	}

	if (!bFoundSect)
	{
		fprintf(ftemp, "\n[%s]\n", szSect);
	}
	
	if (bFoundSect && (!bFoundKey) && bEoff)
	{
		fprintf(ftemp,"\n");
	}
	
	if (!bFoundKey)
	{
		fprintf(ftemp,"%s=",szKey);
	}
	
	fprintf(ftemp, "%s", szValue);

	if (bFoundKey)
	{
		FindNextLine(m_CurLine, m_CurCharInd);
	}
	
	if (!(bEoff || feof(f))) 
	{
		fprintf(ftemp,"\n");
		
		while (!feof(f))
		{
			fscanf(f,"%c",&c);
		  	if (!feof(f))
			{
				fprintf(ftemp,"%c",c);
			}
		}
	}
	
	// fflush(ftemp);
	fpos = ftell(ftemp);
	fclose(f);

	if ((f = fopen(m_fileName.c_str(),"w")) == NULL)
	{
		if ((f = fopen(m_fileName.c_str(),"r")) != NULL)
		{
			fclose(f);
			LOGERROR("INI-File is write protected " << m_fileName.c_str() << endl);
			return -1;
		}

		LOGERROR("INI-File not found " << m_fileName.c_str() << endl);
		return -1;
	}
	
	fseek(ftemp,0,SEEK_SET);
	
	for (i=0;i<fpos;i++)
	{
		fscanf(ftemp,"%c",&c);
		fprintf(f,"%c",c);
	}
	
	fclose(f);
	fclose(ftemp);
	// _rmtmp(); unter UNIX automatisch
	
	return 0;

}

//--------------------------------------------------------------------------------
int IniFile::WriteKeyBool(const char* pSect, const char* pKey, bool bValue)
{
	if(bValue)
	{
		return WriteKeyValue(pSect, pKey, "true");
	}
	else
	{
		return WriteKeyValue(pSect, pKey, "false");
	}
}

//--------------------------------------------------------------------------------
int IniFile::WriteKeyInt(const char* szSect,const char* szKey,int nValue)
{
	char buff[20];
	snprintf(buff, 10, "%d", nValue);
	
	return WriteKeyValue(szSect, szKey, buff);
}

//--------------------------------------------------------------------------------
int IniFile::WriteKeyDouble(	const char* szSect,
								const char* szKey,
								double Value,
								int Length,
								int decimals)
{
	char buff[100];
	sprintf(buff, "%g", Value);
	
	return WriteKeyValue(szSect,szKey, buff);
}
*/
//--------------------------------------------------------------------------------
int IniFile::GetKeyBool(	const char* pSect,
							const char* pKey,
							bool* pValue, 
							bool bWarnIfNotfound)
{
	std::string strRead;
	char pBuf[20];
	*pValue = false;
	if (GetKeyValue(pSect, pKey, pBuf, 20, bWarnIfNotfound) == -1)
	{
		return -1;
	}

	char* pChar = pBuf;
	while( *pChar == ' ' )	pChar++;		// skip spaces

	if( strncmp(pChar, "true", 4) == 0 )		// is the same?
	{
		*pValue = true;
		return 0;
	}
	if( strncmp(pChar, "false", 5) == 0 )		// is the same?
	{
		*pValue = false;
		return 0;
	}

	return -1;
}

//--------------------------------------------------------------------------------
int IniFile::GetKeyInt(	const char* szSect,
						const char* szKey,
						int* pValue, 
						bool bWarnIfNotfound)
{
	char buf[9];
	if (GetKeyValue(szSect, szKey, buf, 9, bWarnIfNotfound) !=-1)
	{
		*pValue = atoi(buf);
		return 0;
	}
	else 
	{
		return -1;
	}
}

//--------------------------------------------------------------------------------
int IniFile::GetKeyLong(	const char* szSect,
							const char* szKey,
							long* pValue, 
							bool bWarnIfNotfound)
{
	char buf[9];
	
	if (GetKeyValue(szSect,szKey,buf,9, bWarnIfNotfound)!=-1)
	{
		*pValue = atol(buf);
		return 0;
	}
	else
	{
		return -1;
	}
}

//--------------------------------------------------------------------------------
int IniFile::GetKeyDouble(	const char* szSect,
							const char* szKey,
							double* pValue, 
							bool bWarnIfNotfound)
{
	char buf[25];
	
	if (GetKeyValue(szSect, szKey, buf, 25, bWarnIfNotfound) == -1)
	{
		std::cout << "Setting parameter " << szKey <<" = " << *pValue << " of section '" << szSect << 
											"' in File '" << m_fileName.c_str();
		return -1;
	}

	*pValue = atof(buf);
	return 0;
}
 
//-----------------------------------------------------------------------------
int IniFile::GetKeyDouble(	const char* pSect,
							const char* pKey,
							double* pValue,
						  	double dDefault,
							bool bWarnIfNotfound)
{
	(*pValue) = dDefault;
	return GetKeyDouble(pSect, pKey, pValue, bWarnIfNotfound);
}

//--------------------------------------------------------------------------------
int IniFile::GetKeyValue(	const char* szSect,
							const char* szKey,
							char* szBuf,
							int lenBuf,
							bool bWarnIfNotfound)
{
	if (!m_bFileOK)
	{
		return -1;
	}

	int   lS,lK;

	lS = strlen(szSect);
	lK = strlen(szKey);
	
	if ((lS * lK) == 0)
	{
		return -1;
	}
	
	if ((f = fopen(m_fileName.c_str(),"r")) == NULL)
	{
		std::cout << "INI-File not found " << m_fileName.c_str();
		return -1;
	}
	
	if ( FindSection(szSect, bWarnIfNotfound) )
	{
		fclose(f); 
		return -1;
	}
	
	if ( FindKey(szSect, szKey, bWarnIfNotfound) )
	{
		fclose(f); 
		return -1;
	}
	
	if (feof(f))
	{
		fclose(f);
		return -1;
	}

	//----------- read szBuf bytes from file into szKey
	int BytesRead = fread( szBuf, 1, lenBuf, f );

	// terminate string
	int StrLen;
	if(BytesRead < lenBuf)
	{
		if( BytesRead == 0 && (!feof(f)) )
		{
			LOGINFO("file read" << endl);
		}
		StrLen = BytesRead;
	}
	else
	{	//assert(BytesRead == lenBuf);
		StrLen = lenBuf-1;
	}
	szBuf[StrLen] = '\0';	
	
	fclose(f);
	return StrLen;
}

//--------------------------------------------------------------------------------
int IniFile::GetKeyString(	const char* szSect,
							const char* szKey,
							std::string* pStrToRead, 
							bool bWarnIfNotfound)
{
	if (!m_bFileOK)
	{
		return -1;
	}

	int   lS,lK;

	lS = strlen(szSect);
	lK = strlen(szKey);
	if ((lS * lK) == 0) return -1;
	if ((f = fopen(m_fileName.c_str(),"r")) == NULL)
	{
		std::cout << "INI-File not found " << m_fileName.c_str() << endl;
		return -1;
	}
	
	if ( FindSection(szSect, bWarnIfNotfound) )
	{
		fclose(f); 
		return -1;
	}
	
	if ( FindKey(szSect, szKey, bWarnIfNotfound) )
	{
		fclose(f); 
		return -1;
	}
	
	if (feof(f))
	{
		fclose(f);
		return -1;
	}

	//----------- read szBuf bytes from file into szKey
	int res = SkipLineUntil(f, '"');			// find beginning of string
	if(res == -1)
	{
		if(bWarnIfNotfound)
		{
			std::cout  << "GetKeyString section " << szSect << " key " << szKey << " first \" not found" << endl;
		}
		fclose(f);
		return -1;
	}

	std::string strRead;
	res = ReadLineUntil(f, '"', strRead);	// read string
	if(res == -1)
	{
		if(bWarnIfNotfound)
		{	
			std::cout << "GetKeyString section " << szSect << " key " << szKey << " string not found" << endl;
		}
		fclose(f);
		return -1;
	}

	// success
	*pStrToRead = strRead;
	fclose(f);
	return 0;
}

//--------------------------------------------------------------------------------
int IniFile::SkipLineUntil(FILE* pFile, const char EndChar)
{
	//assert(pFile);
	int CharsRead = 0;
	while (1)
	{
		int Char = fgetc(pFile);
		
		if (Char == EndChar)			// end found?
		{
			return CharsRead;			// read finished
		}
	
		if (Char == EOF || Char == '\n')
		{
			return -1;				// end not found
		}

		CharsRead++;
	}
}

//--------------------------------------------------------------------------------
int IniFile::ReadLineUntil(FILE* pFile, const char EndChar, std::string& ReadIntoStr)
{
	//assert(pFile);
	int CharsRead = 0;
	while (1)
	{
		int Char = fgetc(pFile);
		
		if (Char == EndChar)			// end found?
		{
			return CharsRead;			// read finished
		}
	
		if (Char == EOF || Char == '\n')
		{
			return -1;				// end not found
		}

		ReadIntoStr.append(1, char(Char));		// todo: not very efficient?

		CharsRead++;
	}
}

//--------------------------------------------------------------------------------
int IniFile::FindNextLine(std::vector<char>& NewLine, int& CharInd)
{
	if (!feof(f))
	{
		fgets(&NewLine[0], NewLine.size(), f); // store next line in NewLine
		CharInd=0;        // makes CharInd reference the first char of the line
        return 0;
	}
	return -1;
}

//--------------------------------------------------------------------------------
int IniFile::FindSection(const char* sect, bool bWarnIfNotfound)
{
	int   lS;
	lS = strlen(sect);
	if (feof(f)) return -1;
  
	FindNextLine(m_CurLine, m_CurCharInd);           //read first line of file
	do
	{
		if (m_CurLine[m_CurCharInd] == '[')
		{
			m_CurCharInd++;
			
			if ((strncmp(&m_CurLine[m_CurCharInd], sect, lS) == 0) && (m_CurLine[m_CurCharInd+lS] == ']')) // if found section name equals searched one
	        { 
				return 0;
			}
			else 
			{
				FindNextLine(m_CurLine, m_CurCharInd);
			}
		}
		else if (m_CurLine[m_CurCharInd] == ' ') // if a blank is found
		{
			m_CurCharInd++;
		}
		else 
		{
			FindNextLine(m_CurLine, m_CurCharInd);
		}
	}
	while (!feof(f));/*do */

	// not found
	if(bWarnIfNotfound)
	{
		std::cout << "Section [" << sect << "] in IniFile " << m_fileName.c_str() << " used by "
			<< m_strIniFileUsedBy << " not found" << endl;

		char c;
		std::cin >> c;
		exit(-1);
	}
	
	return -1;
}

//--------------------------------------------------------------------------------
int IniFile::FindKey(const char* sSection, const char* sKey, bool bWarnIfNotfound)
{
	int   lS;
	long  fpos = 0l;
	lS = strlen(sKey);
	if (feof(f)) return -1;
	
	do
	{
		fpos=ftell(f);             // pointer to the begin of the last read line
		FindNextLine(m_CurLine, m_CurCharInd);
		
		while ( m_CurLine[m_CurCharInd] == ' ' ) // skip blanks
		{
			m_CurCharInd++;
			fpos++;
		}
		
		if (m_CurLine[m_CurCharInd] == '[')		// next section?
		{
			break;		// not found
		}

		if (strncmp(&m_CurLine[m_CurCharInd], sKey, lS) == 0)//Found
		{
			m_CurCharInd+=lS;
			fpos+=lS;                 // set file pointer to end of found key

			while ( m_CurLine[m_CurCharInd] == ' ' ) // skip blanks
			{
				m_CurCharInd++;
				fpos++;
			}
			
			if ( m_CurLine[m_CurCharInd] == '=' )
			{
				m_CurCharInd++;          // set index to first char after the =
				fpos++;
				fseek(f,fpos,SEEK_SET);// set file pointer to first char after the =
				return 0;
			}
			
		}
		
	}
	while (!feof(f));/*do */

	if(bWarnIfNotfound)
	{
		std::cout 	<< "Key:" << sKey << " in Section:" << sSection << " in IniFile:" << m_fileName.c_str()
					<< " used by " << m_strIniFileUsedBy << " not found" << endl;
		char c;
		std::cin >> c;
		exit(-1);
	}
	return -1;
}

//-----------------------------------------------
int IniFile::GetKey(const char* pSect,const char* pKey, std::string* pStrToRead, bool bWarnIfNotfound)
{
	return GetKeyString(pSect,pKey,pStrToRead,bWarnIfNotfound);
}

//-----------------------------------------------
int IniFile::GetKey(const char* pSect,const char* pKey,int* pValue, bool bWarnIfNotfound)
{
	return GetKeyInt(pSect,pKey,pValue,bWarnIfNotfound);
}

//-----------------------------------------------

int IniFile::GetKey(const char* pSect, const char* pKey, bool* pValue, bool bWarnIfNotfound)
{
	return GetKeyBool(pSect,pKey,pValue,bWarnIfNotfound);
}

//-----------------------------------------------
int IniFile::GetKey(const char* pSect,const char* pKey,double* pValue, bool bWarnIfNotfound)
{
	return GetKeyDouble(pSect,pKey,pValue,bWarnIfNotfound);
}

