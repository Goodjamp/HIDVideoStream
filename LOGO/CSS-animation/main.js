$(document).ready(function(){
    $("#a1").click(function(){
        $("#b1").css({
            'display': 'block',
        });
    });
    $("#a2").click(function(){
        $("#b2").css({
            'display': 'block',
        });
    });
    $("#a3").click(function(){
        $("#b3").css({
            'display': 'block',
        });
    });
    $("#a4").click(function(){
        $("#b4").css({
            'display': 'block',
        });
    });
    $("#a5").click(function(){
        $("#b1").css({
            'display': 'none',
        });
        $("#b2").css({
            'display': 'none',
        });
        $("#b3").css({
            'display': 'none',
        });
        $("#b4").css({
            'display': 'none',
        });      
    });
});