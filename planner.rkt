#lang dssl2

# Final project: Trip Planner

import cons
import 'project-lib/graph.rkt'
import 'project-lib/dictionaries.rkt'
import 'project-lib/binheap.rkt'
import sbox_hash

let eight_principles = ["Know your rights.",
                        "Acknowledge your sources.",
                        "Protect your work.",
                        "Avoid suspicion.",
                        "Do your own work.",
                        "Never falsify a record or permit another person to do so.",
                        "Never fabricate data, citations, or experimental results.",
                        "Always tell the truth when discussing your work with your instructor."]

### Basic Types ###

#  - Latitudes and longitudes are numbers:
let Lat?  = num?
let Lon?  = num?

#  - Point-of-interest categories and names are strings:
let Cat?  = str?
let Name? = str?

### Raw Item Types ###

#  - Raw positions are 2-element vectors with a latitude and a longitude
let RawPos? = TupC[Lat?, Lon?]

#  - Raw road segments are 4-element vectors with the latitude and
#    longitude of their first endpoint, then the latitude and longitude
#    of their second endpoint
let RawSeg? = TupC[Lat?, Lon?, Lat?, Lon?]

#  - Raw points-of-interest are 4-element vectors with a latitude, a
#    longitude, a point-of-interest category, and a name
let RawPOI? = TupC[Lat?, Lon?, Cat?, Name?]

### Contract Helpers ###

# ListC[T] is a list of `T`s (linear time):
let ListC = Cons.ListC
# List of unspecified element type (constant time):
let List? = Cons.list?

## Structs ##
struct posit:
    let lat: Lat?
    let long: Lon?

struct POI:
    let name: Name?
    let cat: Cat? 
    let posi: posit?
    
struct road:
    let end_1: posit?
    let end_2: posit?
    let dist: pos?
    
interface TRIP_PLANNER:

    # Returns the positions of all the points-of-interest that belong to
    # the given category.
    def locate_all(
            self,
            dst_cat:  Cat?           # point-of-interest category
        )   ->        ListC[RawPos?] # positions of the POIs

    # Returns the shortest route, if any, from the given source position
    # to the point-of-interest with the given name.
    def plan_route(
            self,
            src_lat:  Lat?,          # starting latitude
            src_lon:  Lon?,          # starting longitude
            dst_name: Name?          # name of goal
        )   ->        ListC[RawPos?] # path to goal

    # Finds no more than `n` points-of-interest of the given category
    # nearest to the source position.
    def find_nearby(
            self,
            src_lat:  Lat?,          # starting latitude
            src_lon:  Lon?,          # starting longitude
            dst_cat:  Cat?,          # point-of-interest category
            n:        nat?           # maximum number of results
        )   ->        ListC[RawPOI?] # list of nearby POIs


class TripPlanner (TRIP_PLANNER):
    let _all_poi: HashTable?
    let _ind_to_posit: HashTable?
    let _posit_to_ind: HashTable?
    let _raw_poi: VecC[RawPOI?]
    let _hash_func
    let _ind_to_road: HashTable? # functions as an adjacency list where each position has
                                 # an index, and at each index, there is an association
                                 # list with a distance and the other other position's index
    let posit_num: int?
    let poi_num: int?
    let road_num: int?
    
    
    def __init__(self, roads: VecC[RawSeg?], POIs: VecC[RawPOI?]):
        self._hash_func = make_sbox_hash()
        self._raw_poi = POIs
        self._all_poi = HashTable(POIs.len(), self._hash_func)
        
        let curr = None
        self.posit_num = 0
        self.poi_num = POIs.len()
        self.road_num = roads.len()
        
        for i in POIs:
            curr = self.cook_poi(i)
            self._all_poi.put(curr.name, curr) 
        let ind = 0
        let p1 = None
        let p2 = None
        
        self._posit_to_ind = HashTable(self.posit_num, self._hash_func)
        self._ind_to_posit = HashTable(self.posit_num, self._hash_func)
        self._ind_to_road = HashTable(self.road_num, self._hash_func)
        
        for i in range(roads.len()):
            curr = self.cook_seg(roads[i])
            p1 = curr.end_1
            p2 = curr.end_2
            
            if not self._posit_to_ind.mem?(p1):
                self._posit_to_ind.put(p1, ind)
                self._ind_to_posit.put(ind, p1)
                self._ind_to_road.put(ind, AssociationList())
                ind = ind + 1
                self.posit_num = self.posit_num + 1
            
            if not self._posit_to_ind.mem?(p2):
                self._posit_to_ind.put(p2, ind)
                self._ind_to_posit.put(ind, p2)
                self._ind_to_road.put(ind, AssociationList())
                ind = ind + 1
                self.posit_num = self.posit_num + 1
                
            self._ind_to_road.get(self._posit_to_ind.get(p1)).put(self._posit_to_ind.get(p2), 
                                                                  curr.dist)
            self._ind_to_road.get(self._posit_to_ind.get(p2)).put(self._posit_to_ind.get(p1), 
                                                                  curr.dist)
     
    def cook_poi(self, raw: RawPOI?) -> POI?:
        return POI(raw[3], raw[2], posit(raw[0], raw[1]))
        
    def cook_seg(self, raw: RawSeg?) -> road?:
        let p1 = posit(raw[0], raw[1])
        let p2 = posit(raw[2], raw[3])
        return road(p1, p2, self.measure(p1, p2))
        
    def measure(self, p1: posit?, p2: posit?) -> pos?:
        let x_comp = (p1.lat - p2.lat)**2
        let y_comp = (p1.long - p2.long)**2
        return (x_comp + y_comp).sqrt()
        
    def to_raw_poi(self, cooked: POI?) -> RawPOI?:
        return vec(cooked.posi.lat, cooked.posi.long, cooked.cat, cooked.name)

    def to_raw_posit(self, cooked: posit?) -> RawPos?:
        return [cooked.lat, cooked.long]
        
    def locate_all(self, dst_cat: Cat?) -> ListC[RawPos?]:
        let fil_poi = self._raw_poi.filter(λ x: x[2] == dst_cat)
        let to_pos = AssociationList()
        let cons_pos = None
        let curr_pos = None
        
        for i in fil_poi:
            curr_pos = posit(i[0], i[1])
            
            if not to_pos.mem?(curr_pos):
                to_pos.put(curr_pos, curr_pos)
                cons_pos = cons(self.to_raw_posit(curr_pos), cons_pos)
                
        return cons_pos
        
    def plan_route(self, src_lat: Lat?, src_lon: Lon?, dst_name: Name?) -> ListC[RawPos?]:
        if self._all_poi.mem?(dst_name) == False: return None
        if self._posit_to_ind.mem?(posit(src_lat, src_lon)) == False: 
            error("Starting point does not exist.")
        
        let ind_to_dist = HashTable(self.posit_num, self._hash_func)
        let ind_to_pred = HashTable(self.posit_num, self._hash_func)
        let this_posit = posit(src_lat, src_lon)
        let dest_ind = self._posit_to_ind.get(self._all_poi.get(dst_name).posi)
        let this_ind = self._posit_to_ind.get(this_posit)
        let todo = BinHeap(self.road_num, 
                           λ x, y: ind_to_dist.get(x) < ind_to_dist.get(y))
        let done = HashTable(self.road_num, self._hash_func)
            
        for i in range(self.posit_num):
            if i == this_ind: ind_to_dist.put(i, 0)
            else: ind_to_dist.put(i, inf)
            
            ind_to_pred.put(i, None) 
        
        todo.insert(this_ind)
      
        let v = None
        while todo.len() > 0:
            v = todo.find_min()
            todo.remove_min()
            if done.len() == 0 or done.mem?(v) == False:
                done.put(v, v)
       
                for j in range(self.posit_num):
                    if self._ind_to_road.get(v).mem?(j): 
                        let u = self._ind_to_road.get(v).get(j)
                        if (ind_to_dist.get(v) + u) < ind_to_dist.get(j):
                            ind_to_dist.put(j, ind_to_dist.get(v) + u)
                            ind_to_pred.put(j, v)
                            todo.insert(j)
                            
        if done.mem?(dest_ind) == False: return None
        
        let path = cons(self.to_raw_posit(self._ind_to_posit.get(dest_ind)), None)      
        let pred = ind_to_pred.get(dest_ind)
        
        while pred != None:
            path = cons(self.to_raw_posit(self._ind_to_posit.get(pred)), path)
            pred = ind_to_pred.get(pred)
            
        return path
        
    def find_nearby(self, src_lat: Lat?, src_lon: Lon?, dst_cat: Cat?, n: nat?) -> ListC[RawPOI?]:
        let fil_poi = self._raw_poi.filter(λ x: x[2] == dst_cat)
        
        if fil_poi.len() == 0: return None
        
        let paths = vec(fil_poi.len())
        let poi_rank = BinHeap(fil_poi.len(), λ x, y: paths[x] < paths[y])
        
        for i in range(fil_poi.len()):
            paths[i] = self.plan_route(src_lat, src_lon, fil_poi[i][3])
            if paths[i] != None:
                paths[i] = self.path_len(paths[i])
                poi_rank.insert(i)
            
        let curr = None
        for i in range(n):
            if poi_rank.len() > 0:
                curr = cons(fil_poi[poi_rank.find_min()], curr)
                poi_rank.remove_min()
            else: break
                
        return curr
                
    def path_len(self, path: ListC[RawPos?]) -> pos?:
        let dist = 0
        let curr = path
        while curr.next != None:
            dist = dist + self.measure(posit(curr.data[0], curr.data[1]), \
                                       posit(curr.next.data[0], curr.next.data[1]))
            curr = curr.next
        return dist
        
def my_first_example():
    return TripPlanner([[0,0, 0,1], [0,0, 1,0]],
                       [[0,0, "bar", "The Empty Bottle"],
                        [0,1, "food", "Pierogi"]])

test 'My first locate_all test':
    assert my_first_example().locate_all("food") == \
        cons([0,1], None)
        
def hollywood_studios():
    let hollywood_boulevard = TripPlanner([[0, 0, 1, -1], [0, 0, -1, -1], [-1, -1, 0, -2], 
                                           [1, -1, 0, -2], [0, -2, 1, -3], [0, -2, 0, -4], 
                                           [0, -4, 0, -5], [0, -5, 0, -6], [0, -6, 0, -7], 
                                           [0, -7, -0.5, -8], [0, -7, 1.5, -8.5], [1.5, -8.5, 0, -9], 
                                           [-0.5, -8, 0, -9]], 
                                          [[0, 0, "attraction", "Mickey & Minnie's Runaway Railway"],  
                                           [1, -3, "dining", "The Hollywood Brown Derby"], 
                                           [0, -5, "dining", "The Trolley Car Café"], 
                                           [-0.5, -8, "shopping", "Sid Cahuenga's One-of-a-Kind"], 
                                           [0, -7, "shopping", "Mickey's of Hollywood"], 
                                           [0, -4, "shopping", "Keystone Clothiers"], 
                                           [0, -7, "shopping", "Celebrity 5 & 10"], 
                                           [1.5, -8.5, "shopping", "Oscar's Super Service"]])
                                           
    return hollywood_boulevard
    
def pdf_map():
    let map = TripPlanner([[0, 0, 0, 1], [0, 0, 1, 0], [0, 1, 0, 2], [0, 1, 1, 1], [1, 0, 1, 1], 
                           [1, 1, 1, 2], [0, 2, 1, 2], [1, 2, 1, 3], [1, 3, -0.2, 3.3]],
                           [[0, 0, "food", "Sandwiches"], [0, 1, "food", "Pasta"], 
                           [0, 1, "clothes", "Pants"], [1, 1, "bank", "Local Credit Union"],
                           [1, 3, "bar", "Bar None"], [1, 3, "bar", "H Bar"], [-0.2, 3.3, "food", 
                                                                               "Burritos"]])
    return map
    
def single():
    let s = TripPlanner([[0, 0, 1, 2]], [[0, 0, "salon", "HairTalk"]])
    return s
    
test 'locate_all test':
    let hs = hollywood_studios()
    assert hs.locate_all("attraction") == cons([0, 0], None)
    assert hs.locate_all("dining") == cons([0, -5], cons([1, -3], None))
    assert hs.locate_all("shopping") == cons([1.5, -8.5], cons([0, -4], cons([0, -7], cons([-0.5, 
                                                                                            -8],
                                                                                           None))))
    assert hs.locate_all("academic") == None
    
test 'locate_all acceptance test':
    let map = pdf_map()
    assert map.locate_all("food") == cons([-0.2, 3.3], cons([0, 1], cons([0, 0], None)))  
    assert map.locate_all("bank") == cons([1, 1], None)
    assert map.locate_all("bar") == cons([1, 3], None)
    assert map.locate_all("clothes") == cons([0, 1], None)
    assert map.locate_all("barber") == None
    
test 'single poi locate_all':
    let m = single()
    assert m.locate_all("magic") == None
    assert m.locate_all("salon") == cons([0, 0], None)
    
def double_poi():
    let tp = TripPlanner([[0, 0, 1.5, 0], [1.5, 0, 2.5, 0], [2.5, 0, 3, 0]], 
                         [[1.5, 0, 'bank', 'Union'], [2.5, 0, 'barber', 'Tony']])
                         
    return tp
                         
test '2 POIs, 1 in relevant category':
    let m = double_poi()
    assert m.locate_all('barber') == cons([2.5, 0], None)
    
test '4 POIs, 2 relevant':
    let tp = TripPlanner([[0, 0, 1.5, 0],
                          [1.5, 0, 2.5, 0],
                          [2.5, 0, 3, 0],
                          [4, 0, 5, 0]],
                         [[1.5, 0, 'bank', 'Union'],
                          [3, 0, 'barber', 'Tony'],
                          [4, 0, 'food', 'Jollibee'],
                          [5, 0, 'barber', 'Judy']])
    let result = tp.locate_all('barber')
    assert result == cons([5, 0], cons([3, 0], None))
    
test '3 POIs, 2 at same loc':
    let tp = TripPlanner([[0, 0, 1.5, 0], 
                          [1.5, 0, 2.5, 0], 
                          [2.5, 0, 3, 0], 
                          [4, 0, 5, 0], 
                          [3, 0, 4, 0]],
                         [[1.5, 0, 'bank', 'Union'], 
                          [3, 0, 'barber', 'Tony'], 
                          [5, 0, 'barber', 'Judy'], 
                          [5, 0, 'barber', 'Lily']])
    assert tp.locate_all('barber') == cons([5, 0], cons([3, 0], None))

test 'My first plan_route test':
   assert my_first_example().plan_route(0, 0, "Pierogi") == \
       cons([0,0], cons([0,1], None))
       
test 'pdf plan_route':
    let tp = pdf_map()
    assert tp.plan_route(0, 0, "Sandwiches") == cons([0, 0], None)
    assert tp.plan_route(0, 1, "Sandwiches") == cons([0, 1], cons([0, 0], None))
    assert tp.plan_route(1, 1, "Sandwiches") == cons([1, 1], cons([0, 1], cons([0, 0], None)))
    assert tp.plan_route(1, 1, "Burritos") == cons([1, 1], cons([1, 2], cons([1, 3], 
                                                                             cons([-0.2, 3.3], 
                                                                                  None))))
    assert tp.plan_route(1, 1, "Sushi") == None
    
def disjoint():
    let dj = TripPlanner([[0, 0, 1, 5], [0, 0, 2, 4], [2, 4, 6, 7], [1, 5, 9, 9], [9, 0, 1, 1],
                          [9, 0, 6, 8], [9, 0, 11, 6], [9, 0, 12, 0], [12, 0, 6, 8], [6, 7, 6, 9],
                          [6, 8, 7, 2], [7, 2, 5, 6], [7, 2, 8, 9]], 
                         [[6, 7, "cafe", "Lisa's"], [2, 4, "student center", "Norris"], 
                          [9, 0, "late night", "Fran's"], [6, 9, "dorm", "Slivka"], 
                          [7, 2, "dorm", "Willard"], [5, 6, "dorm", "Plex"], [8, 9, "dorm", "SMQ"]])
    return dj

test 'disjoint plan_route':
    let dj = disjoint()
    assert dj.plan_route(0, 0, "Fran's") == None
    assert dj.plan_route(9, 0, "Lisa's") == None
    assert dj.plan_route(1, 5, "Norris") == cons([1, 5], cons([0, 0], cons([2, 4], None)))
    
test 'My first find_nearby test':
    assert my_first_example().find_nearby(0, 0, "food", 1) == \
        cons([0,1, "food", "Pierogi"], None)
        
test 'pdf find_nearby':
    let tp = pdf_map()
    assert tp.find_nearby(1, 3, "food", 1) == cons([-0.2, 3.3, "food", "Burritos"], None)
    assert tp.find_nearby(0, 2, "food", 1) == cons([0, 1, "food", "Pasta"], None)
    assert tp.find_nearby(0, 2, "food", 2) == cons([0, 0, "food", "Sandwiches"], 
                                                   cons([0, 1, "food", "Pasta"], None))
    assert tp.find_nearby(0, 2, "food", 3) == cons([-0.2, 3.3, "food", "Burritos"], 
                                                   cons([0, 0, "food", "Sandwiches"], 
                                                        cons([0, 1, "food", "Pasta"], None)))
    assert tp.find_nearby(0, 2, "food", 4) == cons([-0.2, 3.3, "food", "Burritos"], 
                                                   cons([0, 0, "food", "Sandwiches"], 
                                                        cons([0, 1, "food", "Pasta"], None)))
    assert tp.find_nearby(0, 2, "bar", 1) == cons([1, 3, "bar", "Bar None"], None)
    assert tp.find_nearby(0, 2, "bar", 2) == cons([1, 3, "bar", "H Bar"], 
                                                  cons([1, 3, "bar", "Bar None"], None))
    assert tp.find_nearby(0, 2, "bar", 3) == cons([1, 3, "bar", "H Bar"], 
                                                  cons([1, 3, "bar", "Bar None"], None))
    assert tp.find_nearby(0, 2, "school", 5) == None
    
test 'disjoint find_nearby':
    let dj = disjoint()
    assert dj.find_nearby(6, 7, "late night", 2) == None
    assert dj.find_nearby(6, 7, "dorm", 4) == cons([6, 9, 'dorm', "Slivka"], None)
    assert dj.find_nearby(12, 0, "dorm", 3) == cons([8, 9, "dorm", "SMQ"], 
                                                    cons([5, 6, "dorm", "Plex"], 
                                                         cons([7, 2, "dorm", "Willard"], None)))
