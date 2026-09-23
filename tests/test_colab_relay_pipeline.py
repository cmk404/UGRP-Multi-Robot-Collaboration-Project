import concurrent.futures
import json
import threading
import time
from unittest.mock import Mock
import pytest
from scripts.colab_relay_pipeline import Pipeline,drained_seen


def request(identity):
    return {'id':identity,'provider':'jev','body':{'model':'jev-1.13.0'},'expires_unix':time.time()+30}


def test_delivery_retry_keeps_exact_provider_response_and_calls_model_once(tmp_path):
    identity='a'*32;calls=[];puts=[]
    class Client:
        def _request(self,method,path,**kwargs):
            if method=='GET':return {'content':json.dumps(request(identity))}
            puts.append(kwargs['json_data'])
            if len(puts)==1:raise OSError('temporary upload failure')
        def close(self):pass
    response={'status':'http_error','http_status':503,'raw_response':'provider failure','latency_s':.2}
    def execute(row,deadline):calls.append(row);return response
    p=Pipeline(Client,execute,'/content/test',tmp_path,time.time()+10,sleep=lambda _:None)
    assert p.handle(identity)['delivered']
    assert len(calls)==1 and len(puts)==2
    assert json.loads(puts[0]['content'])==json.loads(puts[1]['content'])=={'id':identity,'response':response}
    p.close()


def test_slow_download_does_not_block_fast_response_delivery(tmp_path):
    slow='a'*32;fast='b'*32;blocked=threading.Event();release=threading.Event();delivered=threading.Event()
    class Client:
        def _request(self,method,path,**kwargs):
            identity=path.rsplit('/',1)[1][:-5]
            if method=='GET':
                if identity==slow:
                    blocked.set();assert release.wait(3)
                return {'content':json.dumps(request(identity))}
            if identity==fast:delivered.set()
        def close(self):pass
    p=Pipeline(Client,lambda *_:{'status':'ok','body':{'action':'wait'}},'/content/test',tmp_path,time.time()+10)
    with concurrent.futures.ThreadPoolExecutor(2) as pool:
        a=pool.submit(p.handle,slow);assert blocked.wait(1)
        b=pool.submit(p.handle,fast)
        try:assert delivered.wait(1) and b.result()['delivered']
        finally:release.set()
        assert a.result()['delivered']
    assert len(p.clients)==2
    p.close()


def test_takeover_refuses_uncertain_provider_calls(tmp_path):
    identity='a'*32
    (tmp_path/'status.json').write_text(json.dumps({'calls':1,'delivered':0}))
    (tmp_path/(identity+'-request.json')).write_text(json.dumps(request(identity)))
    with pytest.raises(ValueError,match='outstanding'):drained_seen(tmp_path)
    (tmp_path/(identity+'-response.json')).write_text(json.dumps({'id':identity,'response':{}}))
    with pytest.raises(ValueError,match='outstanding'):drained_seen(tmp_path)
    (tmp_path/'status.json').write_text(json.dumps({'calls':1,'delivered':1}))
    assert drained_seen(tmp_path)=={identity}


def test_pooled_contents_reuses_connection_and_closes_it(monkeypatch):
    import sys
    from types import SimpleNamespace
    from scripts.colab_pooled_contents import pooled_factory
    sessions=[]
    class Http:
        def __init__(self):sessions.append(self);self.calls=[];self.closed=False
        def request(self,*args,**kwargs):
            self.calls.append((args,kwargs))
            return SimpleNamespace(status_code=200,raise_for_status=lambda:None,json=lambda:{'ok':True})
        def close(self):self.closed=True
    class Base:
        def __init__(self,state):self.base_url='https://owned.example';self.token='fixture-token'
    monkeypatch.setitem(sys.modules,'requests',SimpleNamespace(Session=Http))
    monkeypatch.setitem(sys.modules,'colab_cli.contents',SimpleNamespace(ContentsClient=Base,get_status_code=lambda r:r.status_code))
    client=pooled_factory(None)
    assert client._request('GET','/content/a')=={'ok':True}
    client._request('PUT','/content/b',json_data={'content':'same bytes'})
    assert len(sessions)==1 and len(sessions[0].calls)==2
    assert sessions[0].calls[1][1]['timeout']==(5,10)
    client.close();assert sessions[0].closed
