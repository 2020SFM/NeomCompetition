# -*- coding: utf-8 -*-
"""
Created on Thu Aug 27 15:08:01 2020

"""

import threading 
import time 
import json
import requests
import math
from collections import defaultdict
import matplotlib.pyplot as plt
import numpy as np

url = "http://46.101.197.246/api/getAllPoints"

def list_duplicates_of(seq,item):
    start_at = -1
    locs = []
    while True:
        try:
            loc = seq.index(item,start_at+1)
        except ValueError:
            break
        else:
            locs.append(loc)
            start_at = loc
    return locs

def get_eges(url):
    with requests.get(url) as response:
        source =  response.content
        data = json.loads(source)
    
    points = []
    point_st = []
    for dic in data:
            points.append(((dic['lat'],dic['lng']),dic['name']))
    
    poinst1 = []
    poinst2 = []
    distances = []
    def calculateDistance(x1,y1,x2,y2):
         dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
         return dist
   # points_t = points
    for p in points:
        p1 = p[0]
        p_nm = p[1]
        for p_t in points:
            p2 = p_t[0]
            p_tnm = p_t[1]
            if p_nm in point_st and  p_tnm in point_st:
                continue
                #d =  0#float('inf')
            else: d = calculateDistance(float(p1[0]), float(p1[1]), float(p2[0]),float(p2[1]))
            #distances.append((p_nm,p_tnm,d))
            poinst1.append(p_nm)
            poinst2.append(p_tnm)
            distances.append(d)
            
    distances_tmp = distances
    distances_tmp = list(filter(lambda x:x !=0,distances_tmp))
    min_ = min(distances_tmp)*17# + max(distances_tmp))/1.7
    for i,d in enumerate(distances):
        if d >= min_:
            distances[i] = 0
            poinst1[i] = 0
            poinst2[i] = 0
    #print(distances)
    distances = list(filter(lambda x:x !=0,distances))
    poinst1 = list(filter(lambda x:x !=0,poinst1))
    poinst2 = list(filter(lambda x:x !=0,poinst2))

    poinst1_ = []
    poinst2_ = []
    distances_ = []
    for ds in distances:
        locs = list_duplicates_of(distances, ds)
        if distances[locs[0]] in distances_ or distances[locs[0]] == 0:
            continue
        else:
            poinst1_.append(poinst1[locs[0]])
            poinst2_.append(poinst2[locs[0]])
            distances_.append(distances[locs[0]])
    
    edges = []
    for p1,p2,d in iter(zip(poinst1_,poinst2_,distances_)):
        edges.append((p1,p2,round(d,3)))
    return points,edges
    
def build_graph(edge_list):
    graph = defaultdict(list)
    seen_edges = defaultdict(int)
    for src, dst, weight in edge_list:
        seen_edges[(src, dst, weight)] += 1
        if seen_edges[(src, dst, weight)] > 1: 
            continue
        graph[src].append((dst, weight))
        graph[dst].append((src, weight))  
    return graph


def dijkstra(graph, src, dst=None):
    nodes = []
    for n in graph:
        nodes.append(n)
        nodes += [x[0] for x in graph[n]]

    q = set(nodes)
    nodes = list(q)
    dist = dict()
    prev = dict()
    for n in nodes:
        dist[n] = float('inf')
        prev[n] = None

    dist[src] = 0

    while q:
        u = min(q, key=dist.get)
        q.remove(u)

        if dst is not None and u == dst:
            return dist[dst], prev

        for v, w in graph.get(u, ()):
            alt = dist[u] + w
            if alt < dist[v]:
                dist[v] = alt
                prev[v] = u

    return dist, prev


def find_path(pr, node):  # generate path list based on parent points 'prev'
    p = []
    while node is not None:
        p.append(node)
        node = pr[node]
    return p[::-1]
    

#-------------------------------------------------------

import random

def run(): 
    global stop_threads 
    while True: 
        #print('thread running') 
        points,edges = get_eges(url)
        g = build_graph(edges)
        
        print("=== Dijkstra ===")
        
        ar = ['A20' ,'A21' , 'A22' , 'A23', 'A24']
        from_ = random.choice(ar) #get random start point
        #from_ = "A02"
        #to_ = "A33" 
        url_2 = "http://46.101.197.246/api/getWaitingOrders"
        with requests.get(url_2) as response:
            source_2 =  response.content
            data_2 = json.loads(source_2)   
        data_2 =  data_2[-1] # -1 mean get the last dict from list of dict  
        to_ = data_2['station_name'] #get station name from json (last dict )
        id_ = data_2['id'] #get id from same api
#-------------------------------------------------------
        url_3 = "http://46.101.197.246/api/getAllAvailableDrones"
        with requests.get(url_3) as response:
            source_3 =  response.content
            data_3 = json.loads(source_3)
        id_drone = []
        for dic_dorn in data_3:
            id_drone.append(dic_dorn['id'])
        id_drone = random.choice(id_drone)
#-------------------------------------------------------    
        d, prev = dijkstra(g,from_,to_) # get path
        path = find_path(prev, to_)
        print("{} -> {}: distance = {}, path = {}".format(from_,to_,d, path))  

#-------------------------------------------------------
        url_post = "http://46.101.197.246/api/setPathForOrder"
        data_post = {'path':path, 
        'id_':id_, 
        'id_drone':id_drone} 
        requests.post(url = url_post, data = data_post) 
#--------------------------------------------------------
        url_post = "http://46.101.197.246/api/getDroneOrderStatus"
        data_post = {
        'drone_id':id_drone} 
        requests.post(url = url_post, data = data_post) 

        
        if stop_threads: 
            break
  
stop_threads = False
t1 = threading.Thread(target = run) 
t1.start() 
time.sleep(1) 
stop_threads = True
t1.join() 
print('thread killed') 