////////////////////////////////////////////////////////////
// - UpdateSolution -
// A tool for generating project and solution files from
// the OSSIM source tree.
//
// Written By: David A. Horner (http://dave.thehorners.com)
// License: LGPL
// $Id: Form1.cs 10417 2007-02-02 15:02:32Z dburken $
////////////////////////////////////////////////////////////

using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.IO;
using System.Drawing;
using System.Text;
using System.Windows.Forms;
using System.Xml;
using System.Collections;

namespace updatesolution
{
    public partial class Form1 : Form
    {
        // maintains the list of extensions we are going to add to projects.
        private ArrayList theExtList = new ArrayList();
        // the applications we found in the tree.
        private ArrayList theAppList = new ArrayList();
        // break up ossim into its internal parts. (I don't like everything just jammed into the project file)
        private Hashtable theOssimList = new Hashtable();
        // the output directory to write all the solution and project files.
        private string theDestDir = "..\\..\\..\\..\\";
        // the top level of the ossim tree relative to our CWD
        const string theTopLevel = "..\\..\\..\\..\\..\\..";
        // the directory to recurse for applications.
        private string theAppPath = theTopLevel+"\\src\\apps";
        // the directory to recurse for OSSIM src.
        private string theOssimSrcPath = theTopLevel+"\\src\\ossim";
        // the directory to recurse for OSSIM inc.
        private string theOssimIncPath = theTopLevel + "\\include\\ossim";
        // the directory to recurse for OSSIM gdal plugin.
        private string theOssimGdalPath = theTopLevel + "\\..\\ossim_plugins\\gdal";
        // the directory to recurse for OSSIM PNG plugin.
        private string theOssimPNGPath = theTopLevel + "\\..\\ossim_plugins\\png";
        
        // the application project template.
        string theAppTemplate = "..\\..\\..\\templates\\appTemplate.vcproj";
        // the plugin GDAL project template.
        string thePluginGdalTemplate = "..\\..\\..\\templates\\pluginGdalTemplate.vcproj";
        // the plugin PNG project template.
        string thePluginPNGTemplate = "..\\..\\..\\templates\\pluginPNGTemplate.vcproj";
        // the static project template.
        string theStaticTemplate = "..\\..\\..\\templates\\staticTemplate.vcproj";
        // the dll project template.
        string theDllTemplate = "..\\..\\..\\templates\\dllTemplate.vcproj";
        
        // the OSSIM pref file.
        string theOssimPref = "";
        private ProjectInfo theOssimDLLPrj = null;
        private ProjectInfo thePluginGdalPrj = null;
        private ProjectInfo thePluginPNGPrj = null;


        public Form1()
        {
            InitializeComponent();
        }

        // convert two absolute paths to a relative path with respect to mainDirPath
        // code quickly copied from:
        // http://www.snippetcenter.org/en/convert-absolute-to-relative-path-with-c-s1901.aspx
        private static string EvaluateRelativePath(string mainDirPath, string absoluteFilePath)
        {
            string[] firstPathParts = mainDirPath.Trim(Path.DirectorySeparatorChar).Split(Path.DirectorySeparatorChar);
            string[] secondPathParts = absoluteFilePath.Trim(Path.DirectorySeparatorChar).Split(Path.DirectorySeparatorChar);
            int sameCounter = 0;
            for (int i = 0; i < Math.Min(firstPathParts.Length, secondPathParts.Length); i++) { 
                if (!firstPathParts[i].ToLower().Equals(secondPathParts[i].ToLower())) { break; }
                sameCounter++;
            } 
            if (sameCounter == 0) { return absoluteFilePath; } 
            string newPath = String.Empty; 
            for (int i = sameCounter; i < firstPathParts.Length; i++) { 
                if (i > sameCounter) { newPath += Path.DirectorySeparatorChar; } newPath += "..";
            } 
            if (newPath.Length == 0) { newPath = "."; }
            for (int i = sameCounter; i < secondPathParts.Length; i++) {
                newPath += Path.DirectorySeparatorChar; 
                newPath += secondPathParts[i]; 
            } 
            return newPath; 
        }

        // write the hackish (I hate VS .sln format!) .sln format to disk.
        // I'm not writing a full solution file out, just enough to get
        // VS up and running.  VS will fill in the missing info automagically.
        private void WriteSolutionFile(string path)
        {
            bool dep = true;
            status.AppendText("Writing solution @ "+path + "\r\n");
            FileStream Fs = new FileStream(path, FileMode.Create,
                                              FileAccess.Write,
                                              FileShare.ReadWrite);
            // create a unicode writer (dos won't cut it)
            TextWriter tw = new StreamWriter(Fs,Encoding.Unicode);

            // write the header.
            tw.WriteLine("");
            tw.WriteLine("Microsoft Visual Studio Solution File, Format Version 9.00");
            tw.WriteLine("# Visual Studio 2005");

            // dependencies
            tw.WriteLine("Project(\"{8BC9CEB8-8B4A-11D0-8D11-00A0C91BC942}\")" +
                         "= \""+theOssimDLLPrj.theName+"\", \""+theOssimDLLPrj.theProjPath+"\", " +
                         "\"{"+theOssimDLLPrj.theGuid+"}\"");

            /*tw.WriteLine("Project(\"{8BC9CEB8-8B4A-11D0-8D11-00A0C91BC942}\")" +
                         "= \"ossim_dll\", \".\\ossim_dll\\ossim_dll.vcproj\", " +
                         "\"{0A1DFEA3-6E30-432C-AA02-313AF18E9188}\"");*/

            // app folder for grouping the applications together.
            tw.WriteLine("Project(\"{2150E333-8FDC-42A3-9474-1A3956D46DE8}\") = \"Apps\", \"Apps\", \"{04B4259A-9680-4155-910C-C63122938012}\"");

            // ossim pref file
            string prefRelPath = EvaluateRelativePath(theDestDir, theOssimPref);
            tw.WriteLine("Project(\"{2150E333-8FDC-42A3-9474-1A3956D46DE8}\") = \"Solution Items\", \"Solution Items\", \"{2367DF4B-8C26-44FB-9BC9-994045D45C6F}\"");
            tw.WriteLine("\tProjectSection(SolutionItems) = preProject");
            tw.WriteLine("\t\t" + prefRelPath + " = " + prefRelPath);
            tw.WriteLine("\tEndProjectSection");
            tw.WriteLine("EndProject");

            // Add each project to the solution and add the deps.
            foreach (ProjectInfo app in theAppList)
            {
                tw.WriteLine("Project(\"{8BC9CEB8-8B4A-11D0-8D11-00A0C91BC942}\")" +
                             "= \"" + app.theName + "\", \"" + app.theProjPath + "\", " +
                             "\"{" + app.theGuid + "}\"");
                if (dep)
                {
                    tw.WriteLine("\tProjectSection(ProjectDependencies) = postProject");
                    tw.WriteLine("\t\t{"+theOssimDLLPrj.theGuid+"} = {"+theOssimDLLPrj.theGuid+"}");
//                    tw.WriteLine("\t\t{0A1DFEA3-6E30-432C-AA02-313AF18E9188} = {0A1DFEA3-6E30-432C-AA02-313AF18E9188}");
                    tw.WriteLine("\tEndProjectSection");
                }
                tw.WriteLine("EndProject");
            }
            // the png plugin project file.
            tw.WriteLine("Project(\"{8BC9CEB8-8B4A-11D0-8D11-00A0C91BC942}\")" +
                         "= \"" + thePluginPNGPrj.theName + "\", \"" + thePluginPNGPrj.theProjPath + "\", " +
                         "\"{" + thePluginPNGPrj.theGuid + "}\"");
            tw.WriteLine("\tProjectSection(ProjectDependencies) = postProject");
            tw.WriteLine("\t\t{" + theOssimDLLPrj.theGuid + "} = {" + theOssimDLLPrj.theGuid + "}");
            tw.WriteLine("\tEndProjectSection");
            tw.WriteLine("EndProject");
            // the gdal plugin project file.
            tw.WriteLine("Project(\"{8BC9CEB8-8B4A-11D0-8D11-00A0C91BC942}\")" +
                         "= \"" + thePluginGdalPrj.theName + "\", \"" + thePluginGdalPrj.theProjPath + "\", " +
                         "\"{" + thePluginGdalPrj.theGuid + "}\"");
                tw.WriteLine("\tProjectSection(ProjectDependencies) = postProject");
                tw.WriteLine("\t\t{"+theOssimDLLPrj.theGuid+"} = {"+theOssimDLLPrj.theGuid+"}");
                tw.WriteLine("\tEndProjectSection");
            tw.WriteLine("EndProject");
            

            tw.WriteLine("Global");
            
            // write the nesting GUIDs to that VS knows which ones to group under our apps folder.
            tw.WriteLine("\tGlobalSection(NestedProjects) = preSolution");
            foreach (ProjectInfo app in theAppList)
            {
                tw.WriteLine("\t\t{" + app.theGuid + "} = {04B4259A-9680-4155-910C-C63122938012}");
            }
            tw.WriteLine("\tEndGlobalSection");            
            tw.WriteLine("EndGlobal");
            // close the stream
            tw.Close();
            Fs.Close();
            status.AppendText("Finished writing solution file." + "\r\n");
        }

        // Process an application by taking the project template and overriding the
        // important GUIDs and names.  Then we add all the proper files in the file
        // section of the project file.
        private void GenerateApplicationProj(string appName,string pathDir)
        {
            // make sure we have absolute path.
            pathDir = System.IO.Path.GetFullPath(pathDir);

            string destDir = System.IO.Path.Combine(theDestDir,"apps\\" + appName);
            string projPath = System.IO.Path.Combine(destDir,appName + ".vcproj");

            Directory.CreateDirectory(destDir);
            status.AppendText("Application: " + appName + " @ (" + pathDir + ")" + "\r\n");

            ProjectInfo project = new ProjectInfo(appName,EvaluateRelativePath(theDestDir, projPath));
            project.FindFiles(pathDir,theExtList);
            theAppList.Add(project);
            
            // load the application project template for changes.
            XmlDocument xDoc = new XmlDocument();
            xDoc.Load(theAppTemplate);
            XmlNodeList xmlProject = xDoc.GetElementsByTagName("VisualStudioProject");
            if (xmlProject.Count>0)
            {
                // Name
                XmlNode Name=xmlProject[0].Attributes.GetNamedItem("Name");
                Name.Value = appName;
                xmlProject[0].Attributes.SetNamedItem(Name);
                // RootNamespace
                XmlNode RootNamespace = xmlProject[0].Attributes.GetNamedItem("RootNamespace");
                RootNamespace.Value = appName;
                xmlProject[0].Attributes.SetNamedItem(RootNamespace);
                // ProjectGUID
                XmlNode ProjectGUID = xmlProject[0].Attributes.GetNamedItem("ProjectGUID");
                ProjectGUID.Value = "{" + project.theGuid + "}";
                xmlProject[0].Attributes.SetNamedItem(ProjectGUID);                
            }
            // Files
            XmlNodeList xmlFiles = xDoc.GetElementsByTagName("Files");
            if (xmlFiles.Count > 0)
            {
                foreach (FileSystemInfo entry in project.theFiles)
                {
                    string relPath = EvaluateRelativePath(destDir, entry.FullName);
                    status.AppendText("+ " + entry.Name+"\r\n");
                    XmlNode fileNode=xDoc.CreateElement("File");
                    XmlAttribute relAttr=xDoc.CreateAttribute("RelativePath");
                    relAttr.Value = relPath;
                    fileNode.Attributes.Append(relAttr);
                    xmlFiles[0].AppendChild(fileNode);
                }
            }

            // save out the resulting XML, we're done! woot.
            FileStream fsxml = new FileStream(projPath, FileMode.Create,
                                              FileAccess.Write,
                                              FileShare.ReadWrite);
            xDoc.Save(fsxml);
            fsxml.Close();
        }

        private void ProcessOssimStatic()
        {
            status.AppendText("**** Processing OSSIM Static ****" + "\r\n");
            status.AppendText("**** OSSIM Static, code started but not finished. ****" + "\r\n");
            return;
            foreach (ProjectInfo prj in theOssimList.Values)
            {
                string destDir = System.IO.Path.Combine(theDestDir, "static"+Path.DirectorySeparatorChar+prj.theName);
                string projPath = System.IO.Path.Combine(destDir, prj.theName + ".vcproj");
                Directory.CreateDirectory(destDir);
                //update the proj path with where we are putting it for solution file.
                prj.theProjPath=EvaluateRelativePath(theDestDir, projPath);
                // load the application project template for changes.
                XmlDocument xDoc = new XmlDocument();
                xDoc.Load(theStaticTemplate);
                XmlNodeList xmlProject = xDoc.GetElementsByTagName("VisualStudioProject");
                if (xmlProject.Count > 0)
                {
                    // Name
                    XmlNode Name = xmlProject[0].Attributes.GetNamedItem("Name");
                    Name.Value = prj.theName;
                    xmlProject[0].Attributes.SetNamedItem(Name);
                    // RootNamespace
                    XmlNode RootNamespace = xmlProject[0].Attributes.GetNamedItem("RootNamespace");
                    RootNamespace.Value = prj.theName;
                    xmlProject[0].Attributes.SetNamedItem(RootNamespace);
                    // ProjectGUID
                    XmlNode ProjectGUID = xmlProject[0].Attributes.GetNamedItem("ProjectGUID");
                    ProjectGUID.Value = "{" + prj.theGuid + "}";
                    xmlProject[0].Attributes.SetNamedItem(ProjectGUID);
                }
                // Files
                XmlNodeList xmlFiles = xDoc.GetElementsByTagName("Files");
                if (xmlFiles.Count > 0)
                {
                    foreach (FileSystemInfo entry in prj.theFiles)
                    {
                        string relPath = EvaluateRelativePath(destDir, entry.FullName);
                        status.AppendText("+ " + entry.Name + "\r\n");
                        XmlNode fileNode = xDoc.CreateElement("File");
                        XmlAttribute relAttr = xDoc.CreateAttribute("RelativePath");
                        relAttr.Value = relPath;
                        fileNode.Attributes.Append(relAttr);
                        xmlFiles[0].AppendChild(fileNode);
                    }
                }
                // save out the resulting XML, we're done! woot.
                FileStream fsxml = new FileStream(projPath, FileMode.Create,
                                                  FileAccess.Write,
                                                  FileShare.ReadWrite);
                xDoc.Save(fsxml);
                fsxml.Close();
            }
            status.AppendText("**** Done Processing OSSIM Static ****" + "\r\n");
        }

        private void ProcessOssimPNGPlugin()
        {
            if (System.IO.Directory.Exists(theOssimPNGPath))
            {
                status.AppendText("- Processing OSSIM PNG Plugin" + "\r\n");
                string name = "ossim_pngplugin";
                string destDir = System.IO.Path.Combine(theDestDir, name);
                string projPath = System.IO.Path.Combine(destDir, name + ".vcproj");
                Directory.CreateDirectory(destDir);
                thePluginPNGPrj = new ProjectInfo(name, EvaluateRelativePath(theDestDir, projPath));
                thePluginPNGPrj.FindFiles(theOssimPNGPath, theExtList);

                // load the application project template for changes.
                XmlDocument xDoc = new XmlDocument();
                xDoc.Load(thePluginPNGTemplate);
                XmlNodeList xmlProject = xDoc.GetElementsByTagName("VisualStudioProject");
                if (xmlProject.Count > 0)
                {
                    // Name
                    XmlNode Name = xmlProject[0].Attributes.GetNamedItem("Name");
                    Name.Value = thePluginPNGPrj.theName;
                    xmlProject[0].Attributes.SetNamedItem(Name);
                    // RootNamespace
                    XmlNode RootNamespace = xmlProject[0].Attributes.GetNamedItem("RootNamespace");
                    RootNamespace.Value = thePluginPNGPrj.theName;
                    xmlProject[0].Attributes.SetNamedItem(RootNamespace);
                    // ProjectGUID
                    XmlNode ProjectGUID = xmlProject[0].Attributes.GetNamedItem("ProjectGUID");
                    ProjectGUID.Value = "{" + thePluginPNGPrj.theGuid + "}";
                    xmlProject[0].Attributes.SetNamedItem(ProjectGUID);
                }
                // Files
                XmlNodeList xmlFiles = xDoc.GetElementsByTagName("Files");
                if (xmlFiles.Count > 0)
                {
                    foreach (FileSystemInfo entry in thePluginPNGPrj.theFiles)
                    {
                        string relPath = EvaluateRelativePath(destDir, entry.FullName);
                        status.AppendText("+ " + entry.Name + "\r\n");
                        XmlNode fileNode = xDoc.CreateElement("File");
                        XmlAttribute relAttr = xDoc.CreateAttribute("RelativePath");
                        relAttr.Value = relPath;
                        fileNode.Attributes.Append(relAttr);
                        xmlFiles[0].AppendChild(fileNode);
                    }
                }

                // save out the resulting XML, we're done! woot.
                FileStream fsxml = new FileStream(projPath, FileMode.Create,
                                                  FileAccess.Write,
                                                  FileShare.ReadWrite);
                xDoc.Save(fsxml);
                fsxml.Close();
            }
            else
            {
                status.AppendText("- OSSIM PNG source not found, please check it out and try again." + "\r\n");
            }
            status.AppendText("- Done Processing OSSIM PNG Plugin" + "\r\n");
        }

        private void ProcessOssimGdalPlugin()
        {
            if (System.IO.Directory.Exists(theOssimGdalPath))
            {
                status.AppendText("- Processing OSSIM GDAL Plugin" + "\r\n");
                string name = "ossim_gdalplugin";
                string destDir = System.IO.Path.Combine(theDestDir, name);
                string projPath = System.IO.Path.Combine(destDir, name+".vcproj");
                Directory.CreateDirectory(destDir);
                thePluginGdalPrj = new ProjectInfo(name, EvaluateRelativePath(theDestDir, projPath));
                thePluginGdalPrj.FindFiles(theOssimGdalPath, theExtList);

                // load the application project template for changes.
                XmlDocument xDoc = new XmlDocument();
                xDoc.Load(thePluginGdalTemplate);
                XmlNodeList xmlProject = xDoc.GetElementsByTagName("VisualStudioProject");
                if (xmlProject.Count > 0)
                {
                    // Name
                    XmlNode Name = xmlProject[0].Attributes.GetNamedItem("Name");
                    Name.Value = thePluginGdalPrj.theName;
                    xmlProject[0].Attributes.SetNamedItem(Name);
                    // RootNamespace
                    XmlNode RootNamespace = xmlProject[0].Attributes.GetNamedItem("RootNamespace");
                    RootNamespace.Value = thePluginGdalPrj.theName;
                    xmlProject[0].Attributes.SetNamedItem(RootNamespace);
                    // ProjectGUID
                    XmlNode ProjectGUID = xmlProject[0].Attributes.GetNamedItem("ProjectGUID");
                    ProjectGUID.Value = "{" + thePluginGdalPrj.theGuid + "}";
                    xmlProject[0].Attributes.SetNamedItem(ProjectGUID);
                }
                // Files
                XmlNodeList xmlFiles = xDoc.GetElementsByTagName("Files");
                if (xmlFiles.Count > 0)
                {
                    foreach (FileSystemInfo entry in thePluginGdalPrj.theFiles)
                    {
                        string relPath = EvaluateRelativePath(destDir, entry.FullName);
                        status.AppendText("+ " + entry.Name + "\r\n");
                        XmlNode fileNode = xDoc.CreateElement("File");
                        XmlAttribute relAttr = xDoc.CreateAttribute("RelativePath");
                        relAttr.Value = relPath;
                        fileNode.Attributes.Append(relAttr);
                        xmlFiles[0].AppendChild(fileNode);
                    }
                }

                // save out the resulting XML, we're done! woot.
                FileStream fsxml = new FileStream(projPath, FileMode.Create,
                                                  FileAccess.Write,
                                                  FileShare.ReadWrite);
                xDoc.Save(fsxml);
                fsxml.Close();
            }
            else
            {
                status.AppendText("- OSSIM GDAL source not found, please check it out and try again." + "\r\n");
            }
            status.AppendText("- Done Processing OSSIM GDAL Plugin" + "\r\n");
        }

        private void ProcessOssimPlugins()
        {
            status.AppendText("**** Processing OSSIM Plugins ****" + "\r\n");
            ProcessOssimGdalPlugin();
            ProcessOssimPNGPlugin();
            status.AppendText("**** Done Processing OSSIM Plugins ****" + "\r\n");
        }
        private void ProcessOssimDLL()
        {
            status.AppendText("**** Processing OSSIM DLL ****" + "\r\n");
            string destDir = System.IO.Path.Combine(theDestDir, "ossim");
            string projPath = System.IO.Path.Combine(destDir, "ossim.vcproj");
            theOssimDLLPrj = new ProjectInfo("ossim", EvaluateRelativePath(theDestDir, projPath));

            Directory.CreateDirectory(destDir);
            status.AppendText("Writing ossim DLL project @ (" + projPath + ")" + "\r\n");


            // load the application project template for changes.
            XmlDocument xDoc = new XmlDocument();
            xDoc.Load(theDllTemplate);
            XmlNodeList xmlProject = xDoc.GetElementsByTagName("VisualStudioProject");
            if (xmlProject.Count > 0)
            {
                // Name
                XmlNode Name = xmlProject[0].Attributes.GetNamedItem("Name");
                Name.Value = theOssimDLLPrj.theName;
                xmlProject[0].Attributes.SetNamedItem(Name);
                // RootNamespace
                XmlNode RootNamespace = xmlProject[0].Attributes.GetNamedItem("RootNamespace");
                RootNamespace.Value = theOssimDLLPrj.theName;
                xmlProject[0].Attributes.SetNamedItem(RootNamespace);
                // ProjectGUID
                XmlNode ProjectGUID = xmlProject[0].Attributes.GetNamedItem("ProjectGUID");
                ProjectGUID.Value = "{" + theOssimDLLPrj.theGuid + "}";
                xmlProject[0].Attributes.SetNamedItem(ProjectGUID);
            }
            // Files
            XmlNodeList xmlFiles = xDoc.GetElementsByTagName("Files");
            if (xmlFiles.Count > 0)
            {
                foreach (ProjectInfo prj in theOssimList.Values) {

                    XmlNode filterNode = xDoc.CreateElement("Filter");
                    XmlAttribute nameAttr = xDoc.CreateAttribute("Name");
                    nameAttr.Value = prj.theName;
                    filterNode.Attributes.Append(nameAttr);
                    foreach (FileSystemInfo entry in prj.theFiles)
                    {   
                        string relPath = EvaluateRelativePath(destDir, entry.FullName);
                        status.AppendText("+ " + entry.Name + "\r\n");
                        XmlNode fileNode = xDoc.CreateElement("File");
                        XmlAttribute relAttr = xDoc.CreateAttribute("RelativePath");
                        relAttr.Value = relPath;
                        fileNode.Attributes.Append(relAttr);
                        filterNode.AppendChild(fileNode);
                    }
                    xmlFiles[0].AppendChild(filterNode);
                }
            }

            // save out the resulting XML, we're done! woot.
            FileStream fsxml = new FileStream(projPath, FileMode.Create,
                                              FileAccess.Write,
                                              FileShare.ReadWrite);
            xDoc.Save(fsxml);
            fsxml.Close();
            status.AppendText("**** Done Processing OSSIM DLL ****" + "\r\n");
        }
        
        private void FindCoreOSSIMFiles()
        {
            theOssimList.Clear();
            status.AppendText("**** Finding core OSSIM files ****" + "\r\n");
            // Process the directory list for the core src of OSSIM
            DirectoryInfo dir = new DirectoryInfo(theOssimSrcPath);
            foreach (FileSystemInfo entry in dir.GetFileSystemInfos())
            {
                // make sure we are looking at a directory
                if ((entry.Attributes & FileAttributes.Directory) > 0)
                {
                    // don't do anything to svn and CVS directories.
                    if (entry.Name != ".svn" && entry.Name != "CVS")
                    {
                        // process the application folder.
                        string tmpPath = System.IO.Path.Combine(theOssimSrcPath, entry.Name);
                        ProjectInfo project = new ProjectInfo(entry.Name, "");
                        // insert a new project representing this core element
                        // if we already have it, just add the new files.
                        if (!theOssimList.ContainsKey(project.theName))
                        {
                            project.FindFiles(tmpPath, theExtList);
                            theOssimList.Add(project.theName,project);
                        }
                        else
                        {
                            ((ProjectInfo)theOssimList[project.theName]).FindFiles(tmpPath, theExtList);
                            
                        }
                    }
                }
            }

            // Process the directory list for the core inc of OSSIM
            dir = new DirectoryInfo(theOssimIncPath);
            foreach (FileSystemInfo entry in dir.GetFileSystemInfos())
            {
                // make sure we are looking at a directory
                if ((entry.Attributes & FileAttributes.Directory) > 0)
                {
                    // don't do anything to svn and CVS directories.
                    if (entry.Name != ".svn" && entry.Name != "CVS")
                    {
                        // process the application folder.
                        string tmpPath = System.IO.Path.Combine(theOssimIncPath, entry.Name);
                        ProjectInfo project = new ProjectInfo(entry.Name, "");

                        // insert a new project representing this core element
                        // if we already have it, just add the new files.
                        if (!theOssimList.ContainsKey(project.theName))
                        {
                            project.FindFiles(tmpPath, theExtList);
                            theOssimList.Add(project.theName, project);
                        }
                        else
                        {
                            ((ProjectInfo)theOssimList[project.theName]).FindFiles(tmpPath, theExtList);

                        }

                    }
                }
            }
            // tell the user how many files we found
            int total_files = 0;
            foreach (ProjectInfo prj in theOssimList.Values)
            {
                total_files += prj.theFiles.Count;
            }
            status.AppendText("We found "+total_files+" files in the core." + "\r\n");
            status.AppendText("**** Done finding core OSSIM files ****" + "\r\n");
        }

        private void ProcessApplications()
        {
            status.AppendText("**** Processing Applications ****" + "\r\n");
            // Process the directory list for the applications
            DirectoryInfo dir = new DirectoryInfo(theAppPath);
            foreach (FileSystemInfo entry in dir.GetFileSystemInfos())
            {
                // make sure we are looking at a directory
                if ((entry.Attributes & FileAttributes.Directory) > 0)
                {
                    // don't do anything to svn and CVS directories.
                    if (entry.Name != ".svn" && entry.Name != "CVS")
                    {
                        // process the application folder.
                        string tmpPath = System.IO.Path.Combine(theAppPath, entry.Name);
                        GenerateApplicationProj(entry.Name, tmpPath);
                    }
                }
            }
            status.AppendText("**** Done Processing Applications ****" + "\r\n");
        }

        private void printenv()
        {
            status.AppendText("**** Information about env ****" + "\r\n");
            status.AppendText("OSSIM_PREFS_FILE: " + theOssimPref + "\r\n");
            status.AppendText("CWD: " + Directory.GetCurrentDirectory() + "\r\n");
            status.AppendText("Destination Folder: " + theDestDir + "\r\n");
            status.AppendText("Searching App Folder: " + theAppPath + "\r\n");
            status.AppendText("**** Template Information ****" + "\r\n");
            status.AppendText("* Edit the following templates (using a text editor) to change project variables *" + "\r\n");
            status.AppendText("OSSIM App template: " + theAppTemplate + "\r\n");
            status.AppendText("OSSIM DLL template: " + theDllTemplate + "\r\n");
            status.AppendText("OSSIM Static template: " + theStaticTemplate + "\r\n");
            if (System.IO.Directory.Exists(theOssimGdalPath))
            {
                status.AppendText("OSSIM GDAL Plugin template: " + thePluginGdalTemplate + "\r\n");
            }
            else
            {
                status.AppendText("OSSIM GDAL Plugin template: DOESN'T EXIST! - " + thePluginGdalTemplate + "\r\n");
            }
            
        }

        private void process_Click(object sender, EventArgs e)
        {
            // clear previous applications.
            theAppList.Clear();
            // clear previous status messages
            status.Clear();

            printenv();
            FindCoreOSSIMFiles();
            ProcessApplications();
            ProcessOssimDLL();
            ProcessOssimStatic();
            ProcessOssimPlugins();
            status.AppendText("**** Generating Solution File ****" + "\r\n");
            WriteSolutionFile(System.IO.Path.Combine(theDestDir,"solution.sln"));
            status.AppendText("**** Done! ****" + "\r\n");
            status.AppendText("If you have any questions or concerns, contact me through my website's contact page. Thanks." + "\r\n");
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            // make sure we have absolute paths for display
            theOssimPref = System.IO.Path.GetFullPath(Environment.GetEnvironmentVariable("OSSIM_PREFS_FILE"));
            // our search paths
            theAppPath = System.IO.Path.GetFullPath(theAppPath);
            theOssimSrcPath = System.IO.Path.GetFullPath(theOssimSrcPath);
            // the directory to recurse for OSSIM inc.
            theOssimIncPath = System.IO.Path.GetFullPath(theOssimIncPath);
            // the directory to recurse for OSSIM gdal plugin.
            theOssimGdalPath = System.IO.Path.GetFullPath(theOssimGdalPath);
            // where are we writing our results?
            theDestDir = System.IO.Path.GetFullPath(theDestDir);
            // our templates to fill up with the good stuff.
            theAppTemplate = System.IO.Path.GetFullPath(theAppTemplate);
            thePluginGdalTemplate = System.IO.Path.GetFullPath(thePluginGdalTemplate);
            theStaticTemplate = System.IO.Path.GetFullPath(theStaticTemplate);
            theDllTemplate = System.IO.Path.GetFullPath(theDllTemplate);


            // theExtList defines which files we add to the
            // project files as we recurse through the tree.
            theExtList.Add(".cpp");
            theExtList.Add(".cc");
            theExtList.Add(".cxx");
            theExtList.Add(".c");
            theExtList.Add(".h");
            theExtList.Add(".hpp");
            theExtList.Add(".hxx");

            printenv();
        }

        private void linkLabel1_LinkClicked(object sender, LinkLabelLinkClickedEventArgs e)
        {
            System.Diagnostics.Process.Start(linkLabel1.Text);
        }

        private void linkLabel2_LinkClicked(object sender, LinkLabelLinkClickedEventArgs e)
        {
            System.Diagnostics.Process.Start(linkLabel2.Text);
        }
    }
}