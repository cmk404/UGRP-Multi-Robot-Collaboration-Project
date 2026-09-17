"""Static contact solver profiles for the dispatch laboratory.

The reduced mecanum model already owns tyre traction. Global NoSlip also changes
that calibrated drive response. Limit friction solver refinement to declared
finger/cargo pairs while preserving their mixed normal parameters, friction cone,
masses, actuators, collision geometry, and zero adhesion. This is a simulation
profile, not a calibrated hardware claim. See experiments/dispatch-skill-integration-20260917.
"""
from __future__ import annotations
import xml.etree.ElementTree as ET

PROFILES=('legacy','global_noslip','local_contact')


def contact_profile(xml,profile):
    if profile not in PROFILES:raise ValueError('unknown contact solver profile')
    root=ET.fromstring(xml)
    if profile=='global_noslip':root.find('option').set('noslip_iterations','4')
    if profile=='local_contact':
        root.find('option').set('timestep','.0005')
        contact=root.find('contact')
        if contact is None:contact=ET.SubElement(root,'contact')
        def geom(name):return root.find(f'.//geom[@name="{name}"]')
        def values(g,key,default):return list(map(float,g.get(key,default).split()))
        for rid in ('r1','r2','r3'):
            for side in ('left','right'):
                finger=rid+'__'+side+'_finger';a=geom(finger)
                for cargo in ('team_beam_geom','dispatch_box_geom'):
                    b=geom(cargo)
                    if a is None or b is None:raise ValueError('declared contact geometry missing')
                    if int(a.get('priority','0'))!=int(b.get('priority','0')):
                        raise ValueError('profile expects equal existing cargo/finger priorities')
                    fa,fb=values(a,'friction','1 .005 .0001'),values(b,'friction','1 .005 .0001')
                    mu=[max(x,y) for x,y in zip(fa,fb)]
                    friction=[mu[0],mu[0],mu[1],mu[2],mu[2]]
                    weight=float(a.get('solmix','1'))/(float(a.get('solmix','1'))+float(b.get('solmix','1')))
                    def mixed(key,default):
                        x,y=values(a,key,default),values(b,key,default)
                        defaults=list(map(float,default.split()))
                        x=x+defaults[len(x):];y=y+defaults[len(y):]
                        return [weight*u+(1-weight)*v for u,v in zip(x,y)]
                    solref=mixed('solref','.02 1')
                    if min(solref)<=0:raise ValueError('profile expects positive normal solref')
                    attrs={'geom1':finger,'geom2':cargo,
                        'condim':str(max(int(a.get('condim','3')),int(b.get('condim','3')))),
                        'friction':' '.join(map(str,friction)),
                        'solref':' '.join(map(str,solref)),
                        'solimp':' '.join(map(str,mixed('solimp','.9 .95 .001 .5 2'))),
                        # Refine friction damping within the existing Coulomb cone.
                        # The smaller physics step resolves this fast time scale.
                        'solreffriction':'0 -3000','adhesion':'0',
                        'margin':str(float(a.get('margin','0'))+float(b.get('margin','0'))),
                        'gap':str(float(a.get('gap','0'))+float(b.get('gap','0')))}
                    ET.SubElement(contact,'pair',**attrs)
    return ET.tostring(root,encoding='unicode')
