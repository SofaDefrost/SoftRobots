#!/usr/bin/python
# -*- coding: utf-8 -*-
# A small tool to generate .html from the .md contained in the plugin
#
# Syntax:
#   ./buildhtmldocs dirname <hook1.ah> <hook2.ah> <hook3.ah>
#
# The tools scans the dirname to locate all .md
# For each .md it then replace all the ..autolink:: command or ..autofile:: using the replacement rules provided in the .ah files.
# During the ..autolink:: remplacement, the link validity is checked to insure there is no deadlink in the generated documentation.
# When the replacement is done pandoc is called to generate the final.html documentation.
#
# This documentation system should be one day replaced with a full sphinx documentation.
#
# Contributors:
#  	damien.marchal@univ-lille1.fr
import os
import ansicolor
import subprocess
import sys
import re
import ntpath
import json
import re
import urllib.request
import urllib.parse
import io
from bs4 import BeautifulSoup

sofaext=['.scn', ".pyscn", ".psl"]

autofile_re = re.compile("\.\.autofile::(\S*)")

def doAutoFile(aFile, outFile):
    f = open(aFile, "rt")
    fo = open(outFile, "w")

    lineno = 0
    for line in f:
        m = autofile_re.search(line)
        if  m != None:
            path = os.path.dirname(aFile)
            toImportFile = os.path.join(path, m.group(1))

            if not os.path.exists(toImportFile):
                print(ansicolor.red("  -- Missing autofile in line: "+str(lineno) + " => "+toImportFile))
                continue
            f = open(toImportFile, "r")
            fo.write(f.read())
        else:
            fo.write(line)
        lineno+=1

    f.close()
    fo.close()

def replaceStringInFile(aFile, outFile, aDictionary):
        f = io.open(aFile, "rt",encoding='utf8',errors="ignore")
        fo = io.open(outFile, "w",encoding='utf8')

        lineno=1
        for line in f:
                for aString in aDictionary:
                    if aDictionary[aString]["regex"].search(line) != None:
                        url = None
                        validUrl =False
                        exception = None
                        if not "url" in aDictionary[aString]:
                            if "absolutepath" in aDictionary[aString]:
                                commonprefix = os.path.commonprefix([aFile, aDictionary[aString]["absolutepath"]])
                                relpath = os.path.relpath(aDictionary[aString]["absolutepath"], os.path.dirname(aFile))
                                url = relpath
                                validUrl = True
                        else:
                            url = aDictionary[aString]["url"]
                            try:
                                ret = urllib.request.urlopen(url, timeout=10.0)
                                if ret.code == 200:
                                    validUrl = True
                            except Exception as e:
                                exception = e
                        if validUrl:
                            line = aDictionary[aString]["regex"].sub("<a href=\"" + url + "\">" + aDictionary[aString]["name"] + "</a>", line)
                        else:
                            print("  -- Cannot retrieve autolink target '"+str(aDictionary[aString]["name"])+"' in line "+str(lineno)+" pointing to: "+str(url) )
                            print("     possible cause:", exception)
                            line = aDictionary[aString]["regex"].sub(aDictionary[aString]["name"], line)

                m=re.search("..autolink::(.)*", line)
                if m:
                    res = m.group(0)
                    if len(res) > 60:
                        res = res[:60]+" ... "
                    print("  -- Missing autolink in line "+str(lineno)+" : "+res)

                lineno += 1

                #if aString in line:
                #	line=line.replace(aString, "<a href=\"" + dictionary[aString] + "\">" + aString + "</a>")
                fo.write(line)

        fo.close()
        f.close()

if len(sys.argv) <= 2:
        print ("USAGE: ./buildhtmldocs dirname <hook1.ah> <hook2.ah> <hook3.ah>")
        sys.exit(-1)

dictionary={}
print("Loading hooks...")
for hook in sys.argv[2:]:
    if os.path.exists(hook):
        bn = os.path.splitext(os.path.basename(hook))[0]
        print("- Importing "+bn)
        d = json.load(open(hook))
        for dk in d:
            if "ns" not in d[dk]:
                d[dk]["ns"] = bn
            ns = d[dk]["ns"]
            k = d[dk]["ns"]+"::"+dk
            dictionary[k] = d[dk]

            if ns == "":
                dictionary[k]["regex"] = re.compile("\.\.autolink::"+dk+"(?!::)")
            else:
                dictionary[k]["regex"] = re.compile("\.\.autolink::"+ns+"::"+dk+"(?!::)")

            if "url" not in dictionary[k] and "relativepath" in dictionary[k]:
                abspath = os.path.abspath(os.path.dirname(hook))+"/"+dictionary[k]["relativepath"]
                purl = urllib.parse.urlparse(abspath)
                if os.path.exists(purl.path):
                    dictionary[k]["absolutepath"] = abspath
                else:
                    print(ansicolor.red("WARNING: Invalid absolute path... " + purl.path))

            if "desc" not in dictionary[k]:
                dictionary[k]["desc"] = ""

    print(str(len(d))+" hooks loaded.")

def checkAllUrl(dirpath, filename):
    print("  -- Checking of all URL: "+filename)
    soup = BeautifulSoup(open(filename,"r").read(), 'html.parser')
    for link in soup.find_all("a"):
        turl = link.get('href')
        if turl is not None and turl != "javascript:void(0)":
            p = urllib.parse.urlparse(turl)
            if p is not None and len(p.path) != 0:
                if p.netloc == '':
                   turl = "file://" + dirpath + "/" + p.path
                try:
                    r = urllib.request.urlopen(turl,timeout=10)
                except Exception as e:
                    print(ansicolor.red("     Unable to get link " + str(turl) + " \n         because " + str(e)))
    for link in soup.find_all("img"):
        turl = link.get('src')
        if turl is not None and turl != "javascript:void(0)":
            p = urllib.parse.urlparse(turl)
            if p is not None:
                if p.netloc == '':
                   turl = "file://" + dirpath + "/" + p.path
                try:
                    r = urllib.request.urlopen(turl,timeout=10)
                except Exception as e:
                    print(ansicolor.red("     Unable to get image " +  str(turl) + "  \n         because " + str(e)))

pathprefix = os.path.abspath(sys.argv[1]) + "/"
for (dirpath, dirnames, aFilenames) in os.walk(pathprefix):
        dirpath = os.path.abspath(dirpath) + "/"
        for aFilename in aFilenames:
                aFile, ext = os.path.splitext(aFilename)
                if ext in [".md"]:
                        print("Generating: " + os.path.relpath(dirpath + aFile + ".html", pathprefix))
                        os.chdir(dirpath)
                        relpathstyle = os.path.relpath(pathprefix, dirpath)


                        ### AutoFile pass
                        doAutoFile(dirpath + "/" + aFilename, dirpath + "/" + aFile + "_tmp.md" )

                        ### Pandoc pass
                        retcode = subprocess.call(["pandoc", dirpath + "/" +aFile + "_tmp.md", "-s", "-c", relpathstyle+"/docs/style.css", "-o", dirpath + "/" + aFile + ".html.tmp", "--metadata", "pagetitle='"+aFile+"'"])
                        os.remove( dirpath + "/" + aFile + "_tmp.md" )
                        ### Autolink pass
                        if retcode == 0 :
                                replaceStringInFile(dirpath + "/" + aFile + ".html.tmp", dirpath + "/" + aFile + ".html", dictionary)
                                os.remove( dirpath + "/" + aFile + ".html.tmp" )
                                checkAllUrl(dirpath, dirpath + "/" + aFile + ".html")
