#!/usr/bin/python
# -*- coding: utf-8 -*-
# A small tool to generate .html from the .md contains in the plugin
#
# Contributors:
#  	damien.marchal@univ-lille1.fr
#
import os
import subprocess
import sys
import re
import ntpath
import json
import re
import urllib2

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
                print("Missing autofile in line: "+str(lineno) + " => "+toImportFile )
                continue
            f = open(toImportFile, "r")
            fo.write(f.read())
        else:
            fo.write(line)
        lineno+=1

    f.close()
    fo.close()

def replaceStringInFile(aFile, outFile, aDictionary):
        f = open(aFile, "rt")
        fo = open(outFile, "w")

        lineno=1
        for line in f:
                for aString in aDictionary:
                    if aDictionary[aString]["regex"].search(line) != None:
                        url = None
                        validUrl =False
                        if not "url" in aDictionary[aString]:
                            if "absolutepath" in aDictionary[aString]:
                                commonprefix = os.path.commonprefix([aFile, aDictionary[aString]["absolutepath"]])
                                relpath = os.path.relpath(aDictionary[aString]["absolutepath"], os.path.dirname(aFile))
                                url = relpath
                                validUrl = True
                        else:
                            url = aDictionary[aString]["url"]
                            try:
                                ret = urllib2.urlopen(url)
                                if ret.code == 200:
                                    validUrl = True
                            except:
                                pass

                        if validUrl:
                            line = aDictionary[aString]["regex"].sub("<a href=\"" + url + "\">" + aDictionary[aString]["name"] + "</a>", line)
                        else:
                            print("Cannot retrieve autlink target '"+aDictionary[aString]["name"]+"' in line "+str(lineno)+" pointing to: "+url )
                            line = aDictionary[aString]["regex"].sub(aDictionary[aString]["name"], line)

                m=re.search("..autolink::(.)*", line)
                if m:
                    res = m.group(0)
                    if len(res) > 60:
                        res = res[:60]+" ... "
                    print("Missing autolink in line "+str(lineno)+" : "+res)

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

            if "url" not in dictionary[k] and "relativepath" in dictionary[k] :
                abspath = os.path.abspath(os.path.dirname(hook))+"/"+dictionary[k]["relativepath"]
                if os.path.exists(abspath):
                    dictionary[k]["absolutepath"] = abspath
                else:
                    print("WARNING: Invalid absolute path... " + abspath)

            if "desc" not in dictionary[k]:
                dictionary[k]["desc"] = ""

    print(str(len(d))+" hooks loaded.")

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
                        retcode = subprocess.call(["pandoc", dirpath + "/" +aFile + "_tmp.md", "-s", "-c", relpathstyle+"/docs/style.css", "-o", dirpath + "/" + aFile + ".html.tmp"])
                        os.remove( dirpath + "/" + aFile + "_tmp.md" )
                        ### Autolink pass
                        if retcode == 0 :
                                replaceStringInFile(dirpath + "/" + aFile + ".html.tmp", dirpath + "/" + aFile + ".html", dictionary)
                                os.remove( dirpath + "/" + aFile + ".html.tmp" )
