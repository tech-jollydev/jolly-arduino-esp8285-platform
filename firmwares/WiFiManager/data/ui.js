var bnd = function (c, b, a) {
    c.addEventListener(b, a, false)
};
var ubnd = function (c, b, a) {
    c.removeEventListener(b, a, false)
};
var m = function (f, d, g) {
    d = document;
    g = d.createElement("p");
    g.innerHTML = f;
    f = d.createDocumentFragment();
    while (d = g.firstChild) {
        f.appendChild(d)
    }
    return f
};
var $ = function (d, c) {
    d = d.match(/^(\W)?(.*)/);
    return (c || document)["getElement" + (d[1] ? d[1] == "#" ? "ById" : "sByClassName" : "sByTagName")](d[2])
};
var j = function (b) {
    for (b = 0; b < 4; b++) {
        try {
            return b ? new ActiveXObject([, "Msxml2", "Msxml3", "Microsoft"][b] + ".XMLHTTP") : new XMLHttpRequest
        }
        catch (c) {}
    }
};
function domForEach(b, a) {
    return Array.prototype.forEach.call(b, a)
}
e = function (b) {
    return document.createElement(b)
};
function onLoad(b) {
    var a = window.onload;
    if (typeof a != "function") {
        window.onload = b
    }
    else {
        window.onload = function () {
            a();
            b()
        }
    }
}
function addClass(b, a) {
    b.className += " " + a
}
function removeClass(f, c) {
    var b = f.className.split(/\s+/)
        , a = b.length;
    for (var d = 0; d < a; d++) {
        if (b[d] === c) {
            b.splice(d, 1)
        }
    }
    f.className = b.join(" ");
    return b.length != a
}
function toggleClass(b, a) {
    if (!removeClass(b, a)) {
        addClass(b, a)
    }
}
function ajaxReq(h, a, g, c) {
    var f = j();
    f.open(h, a, true);
    var d = setTimeout(function () {
        f.abort();
        console.log("XHR abort:", h, a);
        f.status = 599;
        f.responseText = "request time-out"
    }, 9000);
    f.onreadystatechange = function () {
        if (f.readyState != 4) {
            return
        }
        clearTimeout(d);
        if (f.status >= 200 && f.status < 300) {
            g(f.responseText)
        }
        else {
            console.log("XHR ERR :", h, a, "->", f.status, f.responseText, f);
            c(f.status, f.responseText)
        }
    };
    try {
        f.send()
    }
    catch (b) {
        console.log("XHR EXC :", h, a, "->", b);
        c(599, b)
    }
}
function dispatchJson(f, d, c) {
    var a;
    try {
        a = JSON.parse(f)
    }
    catch (b) {
        console.log("JSON parse error: " + b + ". In: " + f);
        c(500, "JSON parse error: " + b);
        return
    }
    d(a)
}
function ajaxJson(d, a, c, b) {
    ajaxReq(d, a, function (f) {
        dispatchJson(f, c, b)
    }, b)
}
function ajaxSpin(d, a, c, b) {
    //$("#spinner").removeAttribute("hidden");
    ajaxReq(d, a, function (f) {
        //$("#spinner").setAttribute("hidden", "");
        c(f)
    }, function (f, g) {
        //$("#spinner").setAttribute("hidden", "");
        b(f, g)
    })
}
function ajaxJsonSpin(d, a, c, b) {
    ajaxSpin(d, a, function (f) {
        dispatchJson(f, c, b)
    }, b)
}
function hidePopup(a) {
    addClass(a, "popup-hidden");
    addClass(a.parentNode, "popup-target")
}
onLoad(function () {
    var b = $("#layout");
    var f = b.childNodes[0];
    //b.insertBefore(m('<div id="spinner" class="spinner" hidden></div>'), f);
    b.insertBefore(m('<div id="messages"><div id="warning" hidden></div></div><div id="notification" hidden></div>'), f);
    domForEach($(".popup"), function (h) {
        hidePopup(h)
    });
    bnd($("#main"), "click", function () {
        a()
    });
    var a = function () {
        removeClass(b, "active");
    }
});
function setEditToClick(a, b) {
    domForEach($("." + a), function (c) {
        if (c.children.length > 0) {
            domForEach(c.children, function (d) {
                if (d.nodeName === "INPUT") {
                    d.value = b
                }
                else {
                    if (d.nodeName !== "DIV") {
                        d.innerHTML = b
                    }
                }
            })
        }
        else {
            c.innerHTML = b
        }
    })
}
function showSystemInfo(a) {
    Object.keys(a).forEach(function (b) {
        setEditToClick("system-" + b, a[b])
    });
    currAp = a.ssid
}
function getSystemInfo() {
    ajaxJson("GET", "/system/info", showSystemInfo, function (b, a) {
        window.setTimeout(getSystemInfo, 10000)
    })
}
function showWarning(b) {
    var a = $("#wifiwarning");
    a.innerHTML = b;
    a.removeAttribute("hidden");
    //window.scrollTo(0, 0)
}
function hideWarning() {
    if(typeof $("#wifiwarning") != 'undefined' && $("#wifiwarning") != null)
        el = $("#wifiwarning").setAttribute("hidden", "")
}
var notifTimeout = null;
function showNotification(b,c) {
    var a = (typeof c == 'undefined') ? $("#notification") : $("#"+c) ;
    
    a.innerHTML = b;
    a.removeAttribute("hidden");
    if (notifTimeout != null) {
        clearTimeout(notifTimeout)
    }
    notifTimout = setTimeout(function () {
        a.setAttribute("hidden", "");
        notifTimout = null
    }, 4000)
}
function showConfigWiFiMessage() {
    alert("Please connect to an existing wifi to enable this function.")
}
function getBoardInfo() {
	ajaxReq("GET", "boardInfo", function (board) {
		var b = JSON.parse(board);
		$("#version").innerHTML = b.fw_name + " - " + b.fw_version; 
		$("#builddate").innerHTML = b.build_date;
	}, function () {
		console.log("Error during scan")
	});
}
var debug = false;
function consolelog(v, e) {
    if(debug === true) console.log(v);
}
function onchg(t) {
	consolelog('*** onchg start ***');
	var relid = t.getAttribute('Id').split("_");
	var val = t.value;
	//ajax(getUrl()+"/digital/"+relid[1]+"/0");
	$('#digit_'+relid[1]+'_checkbox').checked = false;
	ajax(getUrl()+"/analog/"+relid[1]+"/"+val, relid[0]+"_"+relid[1]+"_value");
	consolelog('*** onchg end ***');
}
function displaypin(t) {
    toggleClass(t,"active");
	toggleClass($("#pin_"+t.getAttribute('Id').split("_")[1]+"_block"),"pin-hidden");
}
function onckl(t) {
	consolelog('*** onckl start ***');
	var relid = t.getAttribute('Id').split("_");
	if (relid[0]==='digit') {
		if (relid[2]=='in')
		{
			ajax(getUrl()+"/mode/"+relid[1]+"/input");
			document.getElementById("digit_"+relid[1]+"_span").innerHTML = '<div class="btnchk"><button id="digital_'+relid[1]+'_but_check" onclick="onckl(this)" class="el_click but_check pure-button button-primary">Check</button></div><div  class="sp_val">Result:<span id="digital_'+relid[1]+'_value"  class="sp_val floatright">0000</span></div>';
		}
		else if (relid[2]=='out')
		{
			ajax(getUrl()+"/mode/"+relid[1]+"/output");
			ajax(getUrl()+"/digital/"+relid[1]+"/0");
			var pwmpins = new Array("3","5","6","9","10","11");
			if (pwmpins.indexOf(relid[1]) == -1)
				document.getElementById("digit_"+relid[1]+"_span").innerHTML = '<div class="btnsld"><div style="padding: 0.2em 1em;"></div><div class="floatleft">Off</div><div class="floatright">On</div><label class="switch"><input id="digit_'+relid[1]+'_checkbox" type="checkbox" onclick="onckl(this)"><span class="slider round"></span></label></div>\n\
																				<div class="sld"></div>';
			else
				document.getElementById("digit_"+relid[1]+"_span").innerHTML = '<div class="btnsld"><div style="padding: 0.2em 1em;"></div><div class="floatleft">Off</div><div class="floatright">On</div><label class="switch"><input id="digit_'+relid[1]+'_checkbox" type="checkbox" onclick="onckl(this)"><span class="slider round"></span></label></div>\n\
																			<div class="sld"><div>PWM ~</div><div id="digital_'+relid[1]+'_value" >0</div><div style="padding: 0.2em 1em;"></div><input id="digital_'+relid[1]+'_pwm" onchange="onchg(this)" type="range"  min="0" max="255" value="0" class=" el_click digit_range"></div>';
		}
		else if (relid[2]=='checkbox' && relid[2]!='pwm')
		{
			if (t.checked) {
				ajax(getUrl()+"/digital/"+relid[1]+"/1");
			}
			else {
				ajax(getUrl()+"/digital/"+relid[1]+"/0"); 
			}
			
		}
	}
	else if (relid[0]==='analog') {
		ajax(getUrl()+"/mode/"+relid[1]+"/input");
		ajax(getUrl()+"/analog/"+relid[1], "analog_"+relid[1]+"_value");
		return ;
	}
	if (relid[3]=='check'){
		ajax(getUrl()+"/digital/"+relid[1], relid[0]+"_"+relid[1]+"_value");
	}
	consolelog('*** onckl start ***');
}   
function getUrl() {
	consolelog('*** getUrl start ***');
	return document.location.origin+":8080/jolly";
}
function ajax(url, elem) {
	ajaxReq("GET", url, function (b) {
		//var c = JSON.parse(b);
		var v = b.split(':');
		if (elem != 'undefined' && elem != null && v[1] != 'undefined')
			$("#"+elem).innerHTML = v[1];
	}, function () {
		console.log("Error")
	});
}
function hide(a) {
	$('#'+a).setAttribute("hidden", "");
}
