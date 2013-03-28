#! /usr/bin/python

#M3 -- Meka Robotics Robot Components
#Copyright (c) 2010 Meka Robotics
#Author: edsinger@mekabot.com (Aaron Edsinger)

#M3 is free software: you can redistribute it and/or modify
#it under the terms of the GNU Lesser General Public License as published by
#the Free Software Foundation, either version 3 of the License, or
#(at your option) any later version.

#M3 is distributed in the hope that it will be useful,
#but WITHOUT ANY WARRANTY; without even the implied warranty of
#MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#GNU Lesser General Public License for more details.

#You should have received a copy of the GNU Lesser General Public License
#along with M3.  If not, see <http://www.gnu.org/licenses/>.

"""
 EEPROM binary reading and writing. (version 2)
"""

from struct import unpack, pack
import pprint

############################
# Main interface functions #
############################

#
# eeprom_to_dict
#
def eeprom_to_dict(in_filename):
    return binary_file_to_dict(in_filename, EEPROM_SPEC)

#
# dict_to_eeprom
#
# will overwrite out_filename if it exists, so be careful.
#
def dict_to_eeprom(dict, out_filename):
    print 'Writing EEPROM ',out_filename
    dict_to_binary_file(dict, EEPROM_SPEC, out_filename)

def pretty_print_dict(d):
    pp = pprint.PrettyPrinter(indent=4)
    pp.pprint(d)
    
debug=0    #switch

############################################################################
# Binary Segment implementations                                           #
#                                                                          #
# A binary segment is a section of binary.  So each segment knows how to   #
# read and write. At the bottom of the file you'll see how we define       #
# EEPROM_SPEC.  A "spec" is just an ordered collection of binary segments. #
############################################################################

class BinarySegment(object):
    def __init__(self, segment_key, word_address = None, byte_address = None):
        self.key = segment_key
        self.word_address = word_address
        self.byte_address = byte_address

    # how many bytes does this segment take up?
    def num_bytes(self):
        return 2

    def read(self, in_port):
        return read_word(in_port)

    def write(self, out_port, value):
        write_word(out_port, value)

class ByteSegment(BinarySegment):
    def num_bytes(self):
        return 1

    def read(self, in_port):
        return read_byte(in_port)

    def write(self, out_port, byte):
        write_byte(out_port, byte)

class ByteArraySegment(BinarySegment):
    def __init__(self, segment_key, array_size, word_address = None, byte_address = None):
        super(ByteArraySegment, self).__init__(segment_key, word_address = word_address, 
                                               byte_address = byte_address)
        self.array_size = array_size

    def num_bytes(self):
        return self.array_size

    def read(self, in_port):
        return read_byte_array(in_port, self.array_size)

    def write(self, out_port, array_of_byte_values):
        write_byte_array(out_port, array_of_byte_values)

class ByteAsNBitChunksSegment(BinarySegment):
    def __init__(self, segment_key, n, word_address = None, byte_address = None):
        super(ByteAsNBitChunksSegment, self).__init__(segment_key, 
                                                      word_address = word_address, 
                                                      byte_address = byte_address)
        self.n = n
    
    def num_bytes(self):
        return 1

    def read(self, in_port):
        return val_to_n_bit_chunks(read_byte(in_port), self.n)

    def write(self, out_port, two_bit_chunks):
        return write_byte(out_port, n_bit_chunks_to_val(two_bit_chunks, self.n))

class SignedWordSegment(BinarySegment):
    def num_bytes(self):
        return 2

    def read(self, in_port):
        return read_signed_word(in_port)

    def write(self, out_port, word):
        write_signed_word(out_port, word)
        
class DoubleWordSegment(BinarySegment):

    def num_bytes(self):
        return 4

    def read(self, in_port):
        r=unpack('I', in_port.read(4))[0]
        debug_bit_count(in_port)
        return r
    
    def write(self, out_port, value):
        out_port.write(pack('I', value))
        debug_bit_count(out_port)
        
class CategoriesSegment(BinarySegment):

    def __init__(self, segment_key, id_to_category_spec):
        super(CategoriesSegment, self).__init__(segment_key)
        self.id_to_category_spec = id_to_category_spec
    
    def num_bytes(self):
        return 0 # we don't validate for categories, so we'll just return 0
    
    def read_category_data(self, cat_type, num_words, in_port):
        if self.id_to_category_spec.has_key(cat_type):
            cat_spec = self.id_to_category_spec[cat_type]
            cat_spec.set_num_words(num_words)
            if debug:
                print 'Cat spec', cat_spec, num_words
            return cat_spec.read(in_port)
        else:
            return map(lambda i: read_byte(in_port), range(0, num_words * 2))

    def write_category_data(self, cat_type, data, out_port):
        if self.id_to_category_spec.has_key(cat_type):
            self.id_to_category_spec[cat_type].write(out_port, data)
        else:
            for byte in data:
                write_byte(out_port, byte)

    def read_category(self, in_port):
        type_and_vendor = read_word(in_port)
        if type_and_vendor == 0xffff:
            return None
        e_2_15 = 2 ** 15
        vendor_specific = type_and_vendor >> e_2_15 # the highest-order bit
        cat_type = (e_2_15 - 1) & type_and_vendor
        num_words_in_cat_data = read_word(in_port)   
        if debug:
            print 'Reading Category of type',cat_type
            print 'Reading Category of size (words)',num_words_in_cat_data
        return { 'Category Type': cat_type,
                 'Vendor Specific': vendor_specific,
                 'Category Data Word Size': num_words_in_cat_data,
                 'Data': self.read_category_data(cat_type, num_words_in_cat_data, 
                                                 in_port) }
    def compute_category_word_size(self,cat_type,data):
        return self.id_to_category_spec[cat_type].compute_word_size(data)
            
    def write_category(self, out_port, cat_dict):
        cat_type = cat_dict['Category Type']
        vendor_specific = cat_dict['Vendor Specific']
        e_2_15 = 2 ** 15
        type_and_vendor = (vendor_specific << e_2_15) | cat_type
        if debug:
            print 'Writing Category of type',cat_type
            print 'Writing Category of size (words)',cat_dict['Category Data Word Size']
        write_word(out_port, type_and_vendor)
        
        nw=self.compute_category_word_size(cat_type,cat_dict['Data'])
        
        if (debug and nw!=cat_dict['Category Data Word Size']):
            print 'Category size for ',cat_type,'does not match, using computed value'
            print 'Computed: ',nw, 'Dictionary',cat_dict['Category Data Word Size']
        write_word(out_port, nw)

        self.write_category_data(cat_type, cat_dict['Data'], out_port)

    def read(self, in_port):
        cat_lst = []
        while True:
            putative_cat = self.read_category(in_port)
            if putative_cat == None:
                break
            else:
                cat_lst.append(putative_cat)
        return cat_lst

    def write(self, out_port, cats):
        for dict in cats:
            self.write_category(out_port, dict)
        # category footer:
        write_word(out_port, 0xffff)

class StringsSegment(BinarySegment):

    def num_bytes(self):
        return 0

    def set_num_words(self, n):
        self.num_words = n

    def read(self, in_port):
        strs = []
        num_strs = read_byte(in_port) # XXX spec says this is a word!
        byte_count = 1
        for i in range(0, num_strs):
            str_len = read_byte(in_port)
            byte_count = byte_count + 1
            new_str = ""
            for j in range(0, str_len):
                c = read_char(in_port)
                byte_count = byte_count + 1
                new_str = new_str + c
            strs.append(new_str)
        if byte_count % 2 == 1: # if odd, then there's one byte of padding
            read_byte(in_port) 
        return strs

    def write(self, out_port, string_lst):
        write_byte(out_port, len(string_lst)) # XXX spec says this is a word, not a byte
        byte_count = 1
        for str in string_lst:
            write_byte(out_port, len(str))
            byte_count = byte_count + 1
            for c in str:
                write_char(out_port, c)
                byte_count = byte_count + 1
        if byte_count % 2 == 1: # if odd, then there's one byte of padding
            write_byte(out_port, 0xff) # we assume padding is always 0xff
            
    def compute_word_size(self,string_list):
        byte_count=1 #num strings
        for str in string_list:
            byte_count=byte_count+1+len(str) #bytes + len
        byte_count=byte_count+byte_count%2 #pad
        return byte_count/2

class CategorySegment(BinarySegment):

    def __init__(self, segment_key):
        super(CategorySegment, self).__init__(segment_key)
        self.num_words = 0 # override if you need it

    def num_bytes(self):
        return 0

    def set_num_words(self, n):
        self.num_words = n

    def read(self, in_port):
        return read_spec(self.gen_spec, in_port)

    def write(self, out_port, dict):
        write_dict_from_spec(dict, self.gen_spec, out_port)
    
    def compute_word_size(self,data):
        n=0
        for g in self.gen_spec:
            n=n+g.num_bytes()
        if n%2>0:
            print 'Error, odd category size for',self.key
            raise ValueError
        return n/2

class GeneralInfoSegment(CategorySegment):
    
    def __init__(self, segment_key):
        super(GeneralInfoSegment, self).__init__(segment_key)
        self.gen_spec = [ ByteSegment('GroupIdx'),
                          ByteSegment('ImgIdx'),
                          ByteSegment('OrderIdx'),
                          ByteSegment('NameIdx'),
                          ByteAsNBitChunksSegment('Physical Layer Ports', 2),
                          ByteSegment('CoE Details'),
                          ByteSegment('FoE Details'),
                          ByteSegment('EoE Details'),
                          ByteSegment('SoEChannels'),
                          ByteSegment('DS402Channels'),
                          ByteSegment('SysmanClass'),
                          ByteSegment('Flags'),
                          SignedWordSegment('CurrentOnEBus'),
                          ByteArraySegment('PAD_Byte', 18)]            

    
class FMMUSegment(CategorySegment):

    # NOTE: Doesn't handle the case where there are not two FMMUs.  If you want to support
    # this, just use self.num_words to determine how many there are.
    
    def __init__(self, segment_key):
        super(FMMUSegment, self).__init__(segment_key)
        self.gen_spec = [ ByteSegment('FMMU0'),
                          ByteSegment('FMMU1') ]

class RepeatedCategorySegment(CategorySegment):

    def __init__(self, segment_key):
        super(RepeatedCategorySegment, self).__init__(segment_key)
        self.element_spec = None # must override this in a sub-class!
        self.each_element_num_words = 0
        
    def num_elements(self):
        return self.num_words/ self.each_element_num_words    
    
    def compute_word_size(self,data):
        num_elements=len(data)-1 #ignore padding
        return num_elements*self.each_element_num_words
    
    def read(self, in_port):
        result = []
        for i in range(0, self.num_elements()):
            dict = read_spec(self.element_spec, in_port)
            result.append(dict)
        return result

    def write(self, out_port, lst_of_dict):
        for d in lst_of_dict:
            if type(d) == dict:
                write_dict_from_spec(d, self.element_spec, out_port)
        
class SyncMSegment(RepeatedCategorySegment):

    def __init__(self, segment_key):
        super(SyncMSegment, self).__init__(segment_key)
        self.each_element_num_words = 4 # (8 bytes)
        self.element_spec = [ BinarySegment('Physical Start Address'),
                              BinarySegment('Length'),
                              ByteSegment('Control Register'),
                              ByteSegment('Status Register'),
                              ByteSegment('Activate'),
                              ByteSegment('PDI CTRL') ]

class TAndRXPDOSegment(CategorySegment):

    def __init__(self, segment_key):
        super(TAndRXPDOSegment, self).__init__(segment_key)
        self.num_pdo=0
        self.num_elements=0
        self.each_element_num_words = 4
        self.pdo_num_words=4
        self.element_spec = [ 
                              BinarySegment('Entry Index'),
                              ByteSegment('Subindex'),
                              ByteSegment('Entry Name Idx'),
                              ByteSegment('Data Type'),
                              ByteSegment('BitLen'),
                              BinarySegment('Flags (2)')]
        
        self.pdo_spec = [ BinarySegment('PDO Index'),
                             ByteSegment('nEntry'),
                             ByteSegment('SyncM'),
                             ByteSegment('Synchronization'),
                             ByteSegment('NameIdx'),
                             BinarySegment('Flags (1)') ]
        
    def read(self, in_port):
        result = []
        self.num_pdo=0
        self.num_elements=0
        while (self.num_pdo*self.pdo_num_words+self.num_elements*self.each_element_num_words)<self.num_words:
            if debug:
                print 'NW Read',self.num_pdo*self.pdo_num_words+self.num_elements*self.each_element_num_words
            pdo = read_spec(self.pdo_spec, in_port)
            result.append(pdo)
            n_entry = pdo['nEntry']
            self.num_pdo=self.num_pdo+1
            for i in range(0, n_entry):
                dict = read_spec(self.element_spec, in_port)
                result.append(dict)
                self.num_elements=self.num_elements+1
        return result
       
    
    def compute_word_size(self,data):
        n_entry=0
        n_pdo=0
        for d in data:
            if type(d)==dict:
                if d.has_key('nEntry'):
                    n_entry=n_entry+d['nEntry']
                    n_pdo=n_pdo+1
        self.num_words=n_pdo*self.pdo_num_words+n_entry*self.each_element_num_words
        return self.num_words
    

    def write(self, out_port, lst_of_dict):
        self.num_pdo=0
        self.num_elements=0
        idx=0
        while (self.num_pdo*self.pdo_num_words+self.num_elements*self.each_element_num_words)<self.num_words:
            write_dict_from_spec(lst_of_dict[idx],self.pdo_spec,out_port)
            n_entry = lst_of_dict[idx]['nEntry']
            idx=idx+1
            self.num_pdo=self.num_pdo+1
            for i in range(0, n_entry):
                write_dict_from_spec(lst_of_dict[idx], self.element_spec, out_port)
                idx=idx+1
                self.num_elements=self.num_elements+1
            
#####################
# Support Functions #
#####################

def debug_bit_count(in_port):
    if debug:
        print 'Byte',in_port.tell()
        
def binary_file_to_dict(filename, spec):
    in_port = open_binary_input_file(filename)
    result = read_spec(spec, in_port)
    in_port.close()
    return result

def dict_to_binary_file(dict, spec, filename):
    out_port = open_binary_output_file(filename)
    write_dict_from_spec(dict, spec, out_port)
    out_port.close()

def read_spec(spec, in_port):
    dict = {}
    byte_count = 0 # for validation
    for bin_seg in spec:
        validate_address_position(bin_seg, byte_count)
        k = bin_seg.key
        if debug:
            print 'Reading segment for ',k
        if dict.has_key(k):
            raise "All spec keys must be unique, but found two of '%s'." % k
        dict[k] = bin_seg.read(in_port)
        byte_count = byte_count + bin_seg.num_bytes()
        if debug:
            print 'Spec read byte count: ',byte_count
    return dict

def write_dict_from_spec(dict, spec, out_port):    
    for bin_seg in spec:
        bin_seg.write(out_port, dict[bin_seg.key])    
                
def open_binary_input_file(filename):
    return open(filename, 'rb')

def open_binary_output_file(filename):
    return open(filename, 'wb')

def read_word(in_port):
    r = in_port.read(2)
    debug_bit_count(in_port)
    if r == '':
        raise "Unexpected end of file reached."
    return unpack('H', r)[0]

def read_signed_word(in_port):
    r = in_port.read(2)
    debug_bit_count(in_port)
    if r == '':
        raise "Unexpected end of file reached."
    return unpack('h', r)[0]

def read_byte(in_port):
    r=unpack('B', in_port.read(1))[0]
    debug_bit_count(in_port)
    return r

def read_char(in_port):
    r=unpack('c', in_port.read(1))[0]
    debug_bit_count(in_port)
    return r

def read_byte_array(in_port, array_size):
    return map(lambda i: read_byte(in_port), range(0, array_size))

def val_to_n_bit_chunks(val, n, total_bits = 8):
    result = []
    mask = (2 ** n) - 1
    for i in range(0, total_bits / n):
        shift = i * n
        result.append((val & (mask << shift)) >> shift)
    return result

def n_bit_chunks_to_val(n_bit_chunk_lst, n):
    val = 0
    for i in range(0, len(n_bit_chunk_lst)):
        val = val | (n_bit_chunk_lst[i] << (i * n))
    return val

def write_word(out_port, word_val):
    out_port.write(pack('H', word_val))

def write_signed_word(out_port, word_val):
    out_port.write(pack('h', word_val))
    
def write_byte(out_port, byte_val):
    out_port.write(pack('B', byte_val))
    debug_bit_count(out_port)
    

def write_byte_array(out_port, byte_array):
    for byte in byte_array:
        write_byte(out_port, byte)

def write_char(out_port, char):
    out_port.write(pack('c', char))
    debug_bit_count(out_port)


def validate_address_position(bin_seg, byte_count):
    wa = bin_seg.word_address
    ba = bin_seg.byte_address
    if wa:
        word_count = byte_count / 2.0
        if wa != word_count:
            raise "At word address 0x%X but expected to be at 0x%X." % (word_count, wa)
    if bin_seg.byte_address:
        if bin_seg.byte_address != byte_count:
            raise "At byte address 0x%X but expected to be at 0x%X." % (byte_count, ba)

#################################################################################
# The EEPROM spec                                                               #
#                                                                               #
# This is basically the program which is interpreted in order to read or write  #
# EEPROM files.                                                                 #
#################################################################################

EEPROM_SPEC = [ 
    # Mandatory Region
    BinarySegment('PDI Control', word_address = 0x0),
    BinarySegment('PDI Configuration', word_address = 0x1),
    BinarySegment('SyncImpulseLen', word_address = 0x2),
    BinarySegment('Extended PDI',  word_address = 0x3),
    BinarySegment('Configured Station Alias', word_address = 0x4),
    ByteArraySegment('Reserved', 4, word_address = 0x5),
    BinarySegment('Checksum', word_address = 0x7),
    
    # Optional Region
    DoubleWordSegment('Vendor ID', word_address = 0x8),
    DoubleWordSegment('Product Code', word_address = 0xA),
    DoubleWordSegment('Revision Number', word_address = 0xC),
    DoubleWordSegment('Serial Number', word_address = 0xE),
    BinarySegment('Execution Delay', word_address = 0x10),
    BinarySegment('Port0 Delay', word_address = 0x11),
    BinarySegment('Port1 Delay', word_address = 0x12),
    BinarySegment('Reserved (2)', word_address = 0x13),
    BinarySegment('Bootstrap Receive Mailbox Offset', word_address = 0x14),
    BinarySegment('Bootstrap Receive Mailbox Size', word_address = 0x15),
    BinarySegment('Bootstrap Send Mailbox Offset', word_address = 0x16),
    BinarySegment('Bootstrap Send Mailbox Size', word_address = 0x17),
    BinarySegment('Standard Receive Mailbox Offset', word_address = 0x18),
    BinarySegment('Standard Receive Mailbox Size', word_address = 0x19),
    BinarySegment('Standard Send Mailbox Offset', word_address = 0x1a),
    BinarySegment('Standard Send Mailbox Size', word_address = 0x1b),
    BinarySegment('Mailbox Protocol', word_address = 0x1c),
    ByteArraySegment('Reserved (3)', 66, word_address = 0x1d),
    BinarySegment('Size', word_address = 0x3e),
    BinarySegment('Version', word_address = 0x3f),
    
    #Categories
    CategoriesSegment('Categories',
                      { 10: StringsSegment('String repository for other Categories'),
                        30: GeneralInfoSegment('General Information'),
                        40: FMMUSegment('FMMUs to be used'),
                        41: SyncMSegment('Sync Manager Configuration'),
                        50: TAndRXPDOSegment('TxPDO description'),
                        51: TAndRXPDOSegment('RxPDO description') })
    ]


    
    