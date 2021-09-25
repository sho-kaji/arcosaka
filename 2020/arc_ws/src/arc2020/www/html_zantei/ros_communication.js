/*
window.onunload = function(){
    Talker.ros.close();
    Listener.ros.close();
};
*/
if(!Talker){
    var Talker = {
        ros : null,
        name : "",
        init : function(){
            this.ros = new ROSLIB.Ros();
            this.ros.on('error', function(error) {
               // document.getElementById('state').innerHTML = "Error";
            });
            this.ros.on('connection', function(error) {
               // document.getElementById('state').innerHTML = "Connect";
            });
            this.ros.on('close', function(error) {
               // document.getElementById('state').innerHTML = "Close";
            });
            //送信トピック作成(ホストネームやポート番号は不明なため暫定で記載)
            this.ros.connect('ws://' + location.hostname + ':9090');
        },

        send : function(user="",pass="",inforeq =false,mainseed="",subseed="",rlen=0,rwidth=0,
                        rheight=0,blueprintreq=false,movestartreq =false,movestopreq=false,continueselect=false){
            //送信トピック作成
            var tp_multi_sniper_login = new ROSLIB.Topic({
                ros : this.ros,
                //暫定で設定
                name :'/sample2/webapp',
                messageType : 'arc2020/webapp'
            });

            //送信メッセージを作成
            var multi_sniper_login = new ROSLIB.Message({
                username         :String(user),
                password         :String(pass),
                inforeq          :Boolean(inforeq),
                mainseed         :String(mainseed),
                subseed          :String(subseed),
                ridge_length     :Number(rlen),
                ridge_width      :Number(rwidth),
                ridge_height     :Number(rheight),
                blueprintreq     :Boolean(blueprintreq),
                movestartreq     :Boolean(movestartreq),
                movestopreq      :Boolean(movestopreq),
                continueselect   :Boolean(continueselect)
            });

            tp_multi_sniper_login.publish(multi_sniper_login);
        }
    }
    Talker.init();
}
if(!Listener){
    var Listener = {
        ros : null,
        sub_to_client: null,
        name : "",
        init : function(){
            this.ros = new ROSLIB.Ros();
            this.ros.on('error', function(error) {
                // document.getElementById('state').innerHTML = "Error";
            });
            this.ros.on('connection', function(error) {
                // document.getElementById('state').innerHTML = "Connect";
            });
            this.ros.on('close', function(error) {
                // document.getElementById('state').innerHTML = "Close";
            });
            this.ros.connect('ws://' + location.hostname + ':9090');
            this.sub_to_client = new ROSLIB.Topic({
                ros : this.ros,
                name : '/sample2/main',
                messageType : 'arc2020/main'
            });
        },
        receive : function(){
            this.sub_to_client.subscribe(function(message) {
                callback(message)
            });
        }        
    }
    Listener.init();
    Listener.receive();
}