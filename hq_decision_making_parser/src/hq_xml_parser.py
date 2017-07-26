#/*************************************************************
# *
# *   hq_xml_parser.py
# *
# *   Adapted for Husqvarna Research Platform
# * 
# *          March 23, 2017
# *     by:  Kent Askenmalm, Husqvarna Group
# *  
# *
# * Code origin: https://github.com/cogniteam/decision_making
# *
# * original header below
# ************************************************************/
 #!/usr/bin/python
import re
import pydot
import xml.etree.ElementTree as ET
from sys import stdout
import ntpath
from optparse import OptionParser
import sys
import os


node_counter=0;
def map_all_ids(xml):
	global node_counter
	map_ids = {}
	for node in xml.iter():
		if "id" not in node.attrib: continue
		map_ids[node.attrib["id"]]=str(node_counter)
		node_counter+=1
	return map_ids

def path_leaf(path):
	head, tail = ntpath.split(path)
	return tail or ntpath.basename(head)
	
class NodeShape:
	def __init__(self, tag_name):
		self.nshape=""
		self.nfillcolor="#FFFFFF"
		self.npenwidth="1"
		if tag_name in ['tsk', 'task']:
			self.nshape="box"
			self.npenwidth="3"
		elif tag_name in ['seq', 'plan', 'pln']:
			self.nshape="rarrow"
		elif tag_name=='par':
			self.nshape="parallelogram"
			self.nfillcolor="#FFCC99"
		else:
			self.nshape="ellipse"
# 			print "###### TAG NAME = ", tag_name
	def set(self, node):
		node.set_shape(self.nshape)
		node.set_fillcolor(self.nfillcolor)
		#node.set_penwidth(self.npenwidth)


def get_tao_start(tao_node):
	return tao_node.attrib["start"]

def get_plan_details(plan_node):
	
	start_condition = "none"
	stop_condition = "none"
	allocate_protocol = "empty"
	next_protocol = "empty"
	
	for subnode in plan_node:
		if subnode.tag == "tao_start_condition":
			start_condition = subnode.text
		if subnode.tag == "tao_stop_condition":
			stop_condition = subnode.text
		if subnode.tag == "tao_allocate":
			allocate_protocol = subnode.attrib["protocol"]
		if subnode.tag == "tao_next":
			next_protocol = subnode.attrib["protocol"]
			
			 
	return (start_condition, stop_condition, allocate_protocol, next_protocol)


def gen_tao(node, graph):
	graph.add_node(pydot.Node("tao_root", label=" ", shape="point"))
	gen_tao_nodes(node, graph)
	graph.add_edge(pydot.Edge("tao_root", get_tao_start(node)))

def gen_tao_nodes(node, graph):
	plans = node[0]
	block = pydot.Subgraph('', rank="same")
	
	for plan in plans:
		(start_condition, stop_condition, allocate_protocol, next_protocol) = get_plan_details(plan)
		import cgi
		 
		label_content = """<
		
		<table style="">
			<tr>
				<td style="background-color: #23a4ff; color: #fff; font-weight: bold; padding: 5px; font-size: 12pt;" align="center">
					<b>{planName}</b>
				</td>
			</tr>
			<tr>
				<td style="padding: 0; maring: 0;">

					<table style="border-width: 1px; border-style: solid; border-color: #999;">
						<tr>
							<td style="padding: 4px; maring: 0;">
			                	{startCondition}
			            	</td>
							<td style="padding: 4px; maring: 0;">
			                	{stopCondition}
			            	</td>
						</tr>
						<tr>
							<td style="padding: 4px; maring: 0;">{allocateProtocol}</td>
							<td style="padding: 4px; maring: 0;">{nextProtocol}</td>
						</tr>
					</table>

				</td>
			</tr>
		</table>
		>""".format(
				planName = plan.attrib["name"], 
				startCondition = cgi.escape(start_condition), 
				stopCondition = cgi.escape(stop_condition), 
				allocateProtocol = allocate_protocol, 
				nextProtocol = next_protocol)
		
		plan_node = pydot.Node(str(plan.attrib["id"]), shape="rectangle", label=label_content, URL=plan.attrib["id"])
		block.add_node(plan_node)
	
	graph.add_subgraph(block)
	gen_tao_edges(node, graph)
	
	for plan in plans:
		for plan_subnode in plan:
			if plan_subnode.tag == 'tao_allocate':
				for subtao in plan_subnode:
					gen_tao_nodes(subtao, graph)
	
	
def gen_tao_edges(node, graph):
	plans = node[0]
	
	#===========================================================================
	# Next edges
	#===========================================================================
	for plan in plans:
		for plan_subnode in plan:
			if plan_subnode.tag == 'tao_next':
				for next_op in plan_subnode:
					edge = pydot.Edge(plan.attrib["id"], next_op.attrib["name"])
					graph.add_edge(edge)
					
	#===========================================================================
	# Alloc edges
	#===========================================================================
	for plan in plans:
		for plan_subnode in plan:
			if plan_subnode.tag == 'tao_allocate':
				for subtao in plan_subnode:
					edge = pydot.Edge(plan.attrib["id"], get_tao_start(subtao))
					graph.add_edge(edge)
		

def graph_gen_nodes(xml, node, graph, elem, ids):
	#print 'proc',node.tag, node.attrib['name'] if 'name' in node.attrib else ""
	if node.tag in ['scxml']:
		for chnode in node:
			graph_gen_nodes(xml, chnode, graph, elem, ids)
	if node.tag in ['parallel']:
		for chnode in node:
			graph_gen_nodes(xml, chnode, graph, elem, ids)
	if node.tag in ['state', 'invoke']:
		if len(node.findall('state')+node.findall('invoke')+node.findall('plan')+node.findall('parallel'))==0:
			
			if node.tag == 'invoke':
				gr_node=pydot.Node(ids[node.attrib["id"]],label=node.attrib["name"], URL=node.attrib["id"], penwidth="0")
			else:
				gr_node=pydot.Node(ids[node.attrib["id"]],label=node.attrib["name"], URL=node.attrib["id"])
			
			elem.add_node(gr_node)
		else:
			has_init_state = 'initialstate' in node.attrib
			lbl = node.attrib["name"]
			if has_init_state : lbl = 'FSM['+lbl+']'
			
			
			gr_cluster=pydot.Cluster(ids[node.attrib["id"]],label=lbl, URL=node.attrib["id"])
			
			elem.add_subgraph(gr_cluster)
			if has_init_state:
				gr_st_node = pydot.Node(ids[node.attrib["id"]]+"start", shape="point")
				gr_cluster.add_node(gr_st_node)
			for chnode in node:
				graph_gen_nodes(xml, chnode, graph, gr_cluster, ids)
	if node.tag in ['plan', 'par', 'seq', 'sel', 'dec', 'pln']:
		nlabel = node.attrib["name"]
		if node.tag in ['plan','pln']: nlabel='BT['+nlabel+']'
		gr_node=pydot.Node(ids[node.attrib["id"]],label=nlabel, URL=node.attrib["id"])
		shape = NodeShape(node.tag)
		shape.set(gr_node)
		elem.add_node(gr_node)
		for chnode in node:
			graph_gen_nodes(xml, chnode, graph, elem, ids)
	if node.tag in ['task','tsk']:
		if len(node.findall('scxml'))==0:
			gr_node=pydot.Node(ids[node.attrib["id"]],label=node.attrib["name"], URL=node.attrib["id"])
			shape = NodeShape(node.tag)
			shape.set(gr_node)
			elem.add_node(gr_node)
			for chnode in node:
				graph_gen_nodes(xml, chnode, graph, elem, ids)
# 			for chnode in node.findall('task'):
# 				graph_gen_nodes(xml, chnode, graph, elem, ids)
		else:
			gr_cluster=pydot.Cluster(ids[node.attrib["id"]],label=node.attrib["name"], URL=node.attrib["id"])
			elem.add_subgraph(gr_cluster)
			for chnode in node.findall('scxml'):
				graph_gen_nodes(xml, chnode, graph, gr_cluster, ids)
				
	#===========================================================================
	# TAO
	#===========================================================================
	
	if node.tag in ['tao']:
		gen_tao(node, graph)
		
# 	if node.tag in ['tao', 'tao_plans']:
# 		for chnode in node:
# 			graph_gen_nodes(xml, chnode, graph, elem, ids)
# 			
# 	if node.tag in ['tao_plan']:
# 		gr_node=pydot.Node(ids[node.attrib["id"]], label=node.attrib["name"], URL=node.attrib["id"])
# 		shape = NodeShape(node.tag)
# 		shape.set(gr_node)
# 		elem.add_node(gr_node)
		

def find_simple_node(state):
	if state.tag == "state" and len([x for x in state if x.tag!='transition'])==0 : return state.attrib["id"]
	if state.tag == "invoke" and len([x for x in state if x.tag!='transition'])==0 : return state.attrib["id"]
	if state.tag in ['plan','pln', 'par', 'dec', 'seq', 'sel' ] : return state.attrib["id"]
	if state.tag in ["task","tsk"] and len(state.findall('scxml'))==0 : return state.attrib["id"]
	for n in state.iter(): 
		if n!=state:
			r = find_simple_node(n)
			if r!=None : return r
	return None

def find_state(fsm, state_id):
	for s in fsm:
		if "id" in s.attrib and s.attrib["id"]==state_id: return s
	return None
	
def graph_gen_edges(xml, node, graph, elem, ids, fsm=None):
	#print 'proc-edge',node.tag, node.attrib['name'] if 'name' in node.attrib else ""
	if node.tag in ['scxml']:
		for chnode in node:
			graph_gen_edges(xml, chnode, graph, elem, ids, node)
	if node.tag in ['parallel']:
		for chnode in node:
			graph_gen_edges(xml, chnode, graph, elem, ids, fsm)
	if node.tag in ['task', 'tsk']:
		for chnode in node:
# 			print "############ SUBPLAN FOUND #################" 
			if "id" in node.attrib and "id" in chnode.attrib:
				src_state_id = node.attrib["id"]
				dst_state_id = chnode.attrib['id']
				src = src_state_id
				dst = dst_state_id
	# 			print "{src} -> {dst}".format(src = ids[src], dst = ids[dst])
				gr_edge = pydot.Edge(ids[src], ids[dst])
				graph.add_edge(gr_edge)
					
# 		for chnode in node.findall('plan'):
# # 			print "############ SUBTASK FOUND #################" 
# 			src_state_id = node.attrib["id"]
# 			dst_state_id = chnode.attrib['id']
# 			src = src_state_id
# 			dst = dst_state_id
# 			print "{src} -> {dst}".format(src = ids[src], dst = ids[dst])
# 			gr_edge = pydot.Edge(ids[src], ids[dst])
# 			graph.add_edge(gr_edge)	
			
		for chnode in node:
			graph_gen_edges(xml, chnode, graph, elem, ids, fsm)
			
	if node.tag in ['state', 'invoke']:
		has_init_state = 'initialstate' in node.attrib
		if has_init_state:
			src_state_id = ids[node.attrib["id"]]+'start'
			dst_state_id = node.attrib["id"]+"/"+node.attrib['initialstate']
			dst_state = find_state(node, dst_state_id)
			dst = None
			if dst_state == None:
				print "ERROR: dst_state == None", dst_state_id
			else:
				dst = find_simple_node(dst_state)
				gr_edge = pydot.Edge(src_state_id, ids[dst])
				if dst!=dst_state_id: gr_edge.set_lhead('cluster_'+ids[dst_state_id])
				#gr_edge.set_ltail(src_state_id)
				gr_edge.set_fontsize("8")
				graph.add_edge(gr_edge)	
		for t in [ t for t in node.findall('transition') if 'target' in t.attrib ]:
			src_state_id = node.attrib["id"]
			dst_state_id = t.attrib['target']
			src = find_simple_node(node)
			dst_state = find_state(fsm, dst_state_id)
			dst = None
			if dst_state == None:
				print "ERROR: dst_state == None"
			else:
				dst = find_simple_node(dst_state)
				gr_edge = pydot.Edge(ids[src], ids[dst])
				gr_edge.set_label(t.attrib["event"])
				if dst!=dst_state_id: gr_edge.set_lhead('cluster_'+ids[dst_state_id])
				if src!=src_state_id: gr_edge.set_ltail('cluster_'+ids[src_state_id])
				gr_edge.set_fontsize("8")
				graph.add_edge(gr_edge)		
		for chnode in node:
			graph_gen_edges(xml, chnode, graph, elem, ids, node)
	if node.tag in ['plan', 'par', 'seq', 'sel', 'dec', 'pln']:
		for chnode in node:
			src_state_id = node.attrib["id"]
			dst_state_id = chnode.attrib['id']
			src = find_simple_node(node)
			dst_state = chnode
			dst = find_simple_node(dst_state)
			gr_edge = pydot.Edge(ids[src], ids[dst])
			
			if dst!=dst_state_id: gr_edge.set_lhead('cluster_'+ids[dst_state_id])


			#gr_edge.set_ltail(ids[src_state_id])
			graph.add_edge(gr_edge)		
			graph_gen_edges(xml, chnode, graph, elem, ids, fsm)
			
			
			
	#===========================================================================
	# TAO
	#===========================================================================
	
	if node.tag in ['tao']:
		for chnode in node:
			graph_gen_edges(xml, chnode, graph, elem, ids, node)
			
	if node.tag in ['tao_plans']:
		for chnode in node:
			graph_gen_edges(xml, chnode, graph, elem, ids, node)
		
				

if __name__ == '__main__':

	parser = OptionParser()
	parser.add_option("-v", "--verbose", dest="verbose",
	                  help="verbose printouts", nargs=0)
	parser.add_option("-q", "--quiet", dest="quite",
	                  help="quiet printouts", nargs=0)
	parser.add_option("-i", "--info", dest="info",
	                  help="printouts just locations info", nargs=0)
	
	(options, args) = parser.parse_args()
	
	if len(sys.argv) < 2:
		sys.exit('Usage: %s projectName destinationFolder [...xmlFiles]' % sys.argv[0])

	if options.verbose is () or options.info is ():
		print "  -- project location is",args[0]
		print "  -- share folder is",args[1]
	if options.verbose is ():
		print "  -- FILES: ",'\n'.join(args[2:])
	print "-- Start decision making xml parsing"
	try:
		for fileXML in args[2].split(";"):
			fileName = path_leaf(fileXML)
			filetype = fileXML[-len("XXxml"):]
			if options.verbose is ():
				print fileXML, fileName,filetype
			xml = ET.parse(fileXML).getroot()
			graph = pydot.Dot(graph_type='digraph', compound='true')
			graph.set_node_defaults(shape="box");
			map_ids = map_all_ids(xml)
			if options.verbose is ():
				for k,v in map_ids.items(): print k,":",v
			graph_gen_nodes(xml, xml, graph, graph, map_ids)
			graph_gen_edges(xml, xml, graph, graph, map_ids)
			
			(fname, fextension) = os.path.splitext(os.path.basename(fileName))
			
			graph.write_raw(args[1] + os.sep + fname + ".dot")
	except:
		print "  -- Unexpected error (",fileXML,"):", sys.exc_info()
		import traceback
		traceback.print_exc()
	print "-- End decision making xml parsing"
