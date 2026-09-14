from pathlib import Path
import math
import xml.etree.ElementTree as E
from matplotlib.textpath import TextPath
from matplotlib.font_manager import FontProperties
from shapely.geometry import Polygon
from shapely.ops import triangulate
p=Path(__file__).resolve().parent
(p/'meshes').mkdir(exist_ok=True)
def obj(name,verts,faces):
 (p/'meshes'/f'{name}.obj').write_text('# Procedural Colin-style simulation prop, dimensions in meters\n'+''.join('v %.7f %.7f %.7f\n'%tuple(v) for v in verts)+''.join('f '+' '.join(str(i+1) for i in f)+'\n' for f in faces))
 return 'model://spray_bottle/meshes/'+name+'.obj'
rings=[(0,.033,.023),(.004,.039,.028),(.012,.042,.029),(.04,.043,.029),(.09,.041,.028),(.125,.038,.026),(.145,.032,.023),(.16,.024,.020),(.178,.016,.015),(.195,.014,.013)]
vs=[]; fs=[]; n=64
for z,rx,ry in rings:
 for j in range(n):
  a=j*2*math.pi/n; c,s=math.cos(a),math.sin(a)
  vs.append((rx*math.copysign(abs(c)**.65,c),ry*math.copysign(abs(s)**.65,s),z))
for r in range(len(rings)-1):
 for j in range(n):
  a=r*n+j; b=r*n+(j+1)%n; c=b+n; d=a+n
  fs.extend([(a,b,c),(a,c,d)])
vs.extend([(0,0,0),(0,0,rings[-1][0])]); bottom=len(vs)-2; top=len(vs)-1
for j in range(n): fs.extend([(bottom,(j+1)%n,j),(top,(len(rings)-1)*n+j,(len(rings)-1)*n+(j+1)%n)])
body=obj('contoured_bottle',vs,fs)
def extrude(name,profile,depth):
 v=[(x,y,z) for y in [-depth/2,depth/2] for x,z in profile]; f=[]; count=len(profile)
 poly=Polygon(profile)
 for t in triangulate(poly):
  if not poly.covers(t): continue
  ids=[profile.index(tuple(pt)) for pt in list(t.exterior.coords)[:3]]
  f.extend([tuple(reversed(ids)),tuple(i+count for i in ids)])
 for i in range(count):
  j=(i+1)%count; f.extend([(i,j,j+count),(i,j+count,i+count)])
 return obj(name,v,f)
head=extrude('sprayer_shell',[(-.027,.210),(-.026,.226),(-.018,.237),(.005,.240),(.047,.233),(.049,.216),(.026,.215),(.012,.207),(-.009,.207)],.027)
trigger=extrude('curved_trigger',[(.021,.217),(.028,.214),(.030,.200),(.028,.183),(.021,.173),(.016,.175),(.021,.188),(.023,.201)],.010)
def textmesh(name,text,width,centerz):
 path=TextPath((0,0),text,size=1,prop=FontProperties(family='DejaVu Sans',weight='bold',style='italic'))
 polys=[Polygon(a) for a in path.to_polygons() if len(a)>2]
 geom=Polygon()
 for poly in polys: geom=geom.symmetric_difference(poly)
 minx,miny,maxx,maxy=geom.bounds; scale=width/(maxx-minx); v=[]; f=[]
 for poly in getattr(geom,'geoms',[geom]):
  for t in triangulate(poly):
   if not poly.covers(t): continue
   ids=[]
   for x,z in list(t.exterior.coords)[:3]:
    ids.append(len(v)); v.append(((x-(minx+maxx)/2)*scale,-.03035,centerz+(z-(miny+maxy)/2)*scale))
   f.extend([tuple(ids),tuple(reversed(ids))])
 return obj(name,v,f)
logo=textmesh('colin_logo','Colin',.058,.099)
subtitle=textmesh('glass_cleaner','GLASS CLEANER',.055,.075)
sdf=E.Element('sdf',version='1.7'); model=E.SubElement(sdf,'model',name='spray_bottle'); E.SubElement(model,'static').text='false'; link=E.SubElement(model,'link',name='bottle')
i=E.SubElement(link,'inertial'); E.SubElement(i,'pose').text='0 0 0.095 0 0 0'; E.SubElement(i,'mass').text='0.55'; tensor=E.SubElement(i,'inertia')
for k,v in dict(ixx=.0018,iyy=.0020,izz=.00048,ixy=0,ixz=0,iyz=0).items(): E.SubElement(tensor,k).text=str(v)
def geom(parent,kind,dims):
 g=E.SubElement(E.SubElement(parent,'geometry'),kind)
 if kind=='mesh': E.SubElement(g,'uri').text=dims
 elif kind=='box': E.SubElement(g,'size').text=' '.join(map(str,dims))
 else:
  E.SubElement(g,'radius').text=str(dims[0]); E.SubElement(g,'length').text=str(dims[1])
def visual(name,kind,dims,color,pose='0 0 0 0 0 0'):
 v=E.SubElement(link,'visual',name=name); E.SubElement(v,'pose').text=pose; geom(v,kind,dims); mat=E.SubElement(v,'material')
 for tag in ['ambient','diffuse']: E.SubElement(mat,tag).text=color
 E.SubElement(mat,'specular').text='0.4 0.4 0.4 1'
def collision(name,kind,dims,pose):
 c=E.SubElement(link,'collision',name=name); E.SubElement(c,'pose').text=pose; geom(c,kind,dims)
 ode=E.SubElement(E.SubElement(E.SubElement(c,'surface'),'friction'),'ode')
 for tag in ['mu','mu2']: E.SubElement(ode,tag).text='0.7'
visual('blue_contoured_body','mesh',body,'0.015 0.30 0.80 1')
visual('white_sprayer','mesh',head,'0.93 0.95 0.98 1')
visual('blue_trigger','mesh',trigger,'0.015 0.07 0.40 1')
visual('blue_screw_cap','cylinder',[.018,.018],'0.015 0.08 0.45 1','0 0 0.201 0 0 0')
visual('blue_nozzle','box',[.015,.030,.023],'0.015 0.08 0.45 1','.049 0 .225 0 0 0')
visual('nozzle_hole','cylinder',[.0025,.0005],'0.01 0.015 0.025 1','.0567 0 .225 0 1.5707963 0')
visual('white_label','box',[.066,.0004,.082],'0.92 0.97 1 1','0 -.030 .087 0 0 0')
visual('colin_red_logo','mesh',logo,'0.90 0.015 0.035 1')
visual('label_text','mesh',subtitle,'0.01 0.11 0.36 1')
visual('label_bottom_band','box',[.062,.0003,.012],'0.015 0.30 0.75 1','0 -.0303 .055 0 0 0')
collision('body','box',[.076,.052,.134],'0 0 .068 0 0 0')
collision('shoulder','box',[.057,.040,.03],'0 0 .145 0 0 0')
collision('neck','cylinder',[.016,.041],'0 0 .178 0 0 0')
collision('cap','cylinder',[.018,.018],'0 0 .201 0 0 0')
collision('head','box',[.081,.030,.028],'.015 0 .224 0 0 0')
collision('trigger','box',[.01,.010,.038],'.024 0 .193 0 0 0')
E.indent(sdf); E.ElementTree(sdf).write(p/'model.sdf',encoding='utf-8',xml_declaration=True)
(p/'model.config').write_text('''<?xml version="1.0"?>
<model><name>Colin Spray Bottle</name><version>2.0</version><sdf version="1.7">model.sdf</sdf><description>Colin-style blue contoured glass-cleaner bottle, white and blue trigger sprayer and red Colin label. Approximate visual prop, 24 cm tall and 550 g.</description></model>
''')
(p/'README.md').write_text('''# Colin-style spray bottle

Replaces the original generic spray bottle. Contoured flattened blue body, tapered shoulder, white sprayer, blue cap/nozzle/curved trigger, and red Colin lettering on a white label. Approximate product-inspired geometry, not an exact manufacturer CAD model. Reference: https://www.pobara.com/colin-glass-cleaner-spray-500ml

Approximately 24 cm tall, 8.6 cm wide, 5.8 cm deep; simulation mass 550 g. Opaque blue material ensures reliable rendering. Rigid prop with simplified compound collision geometry; no trigger articulation or liquid simulation. Origin is at the bottom center; nozzle points +X, label faces -Y.

Gazebo Resource Spawner → Local resources → `/home/ws/assets/models` → **Colin Spray Bottle**. Reopen the panel to refresh. Delete and reinsert any previously spawned generic bottle to see the update. Model URI remains `model://spray_bottle`.

OBJ meshes are local, in meters, and require no textures. `generate_model.py` regenerates the model using matplotlib and shapely.
''')
print('Updated',p)
