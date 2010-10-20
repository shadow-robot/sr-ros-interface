import xml.etree.ElementTree as ET

from Grasp import Grasp

DEBUG = 0

class GraspParser():
    """
    Parses a XML file describing a grasp.
    """
    def __init__(self):
        #initialize stuff
        self.xml_tree = ""
        self.grasps = {}
        self.xml_path = ""
    
    def parse_tree(self, xml_filename="grasps.xml"):
        """
        parses a given tree, returns a Grasp
        
        Keyword arguments:
        xml_filename -- the filename where the grasp is defined
                        if no filename is provided, then the default
                        value is "grasps.xml"
        """    
        #parse the xml tree
        try:
            self.xml_tree = ET.parse(xml_filename)
        except Exception, inst:
            print "Unexpected error opening %s: %s" % (xml_filename, inst)
            return
        
        self.xml_path = xml_filename
        
        tree_root = self.xml_tree.getroot()
        tree_grasp = tree_root.findall("grasp")

        for grasp in tree_grasp:
            grasp_tmp = Grasp()
            grasp_tmp.grasp_name = grasp.attrib.get("name")
            if DEBUG >= 2:
                print "Grasp "+ grasp_tmp.grasp_name
                
            joints = grasp.findall("joint")
            for j in joints:
                joint_name = j.attrib.get("name")
                joint_position = float(j.text)
                if DEBUG >= 2:
                    print "  "+ joint_name + ": "+ str(joint_position)
                
                grasp_tmp.joints_and_positions.update({joint_name:joint_position})                
            
            self.grasps.update({grasp_tmp.grasp_name: grasp_tmp})
            
    def write_grasp_to_file(self, grasp, xml_filename = ""):
        if xml_filename == "":
            xml_filename = self.xml_path
            
        toWrite = grasp.convert_to_xml() 
        objFileRead = open(xml_filename,'r')
        previous = objFileRead.readlines()
        objFileRead.close()
        objFileWrite = open(xml_filename,'w')
        for index in range (0,len(previous)-1):
            objFileWrite.write(previous[index])
        objFileWrite.write(toWrite)
        objFileWrite.write('</root>')
        objFileWrite.close()
        
        self.parse_tree(xml_filename)
    
    def refresh(self):
        self.parse_tree(self.xml_path)
            
############################
#    MAIN - simple test    #
############################
def main():
    parser = GraspParser()
    parser.parse_tree()
    
    return 0


# start the script
if __name__ == "__main__":
    main()
