const char  test_wav[] PROGMEM = {
    0x52, 0x49, 0x46, 0x46, 0x64, 0xC5, 0x66, 0x1B, 0x57, 0x41, 0x56, 0x45, 0x66, 0x6D, 0x74, 0x20,
    0x10, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x40, 0x1F, 0x00, 0x00, 0x80, 0x3E, 0x00, 0x00,
    0x02, 0x00, 0x10, 0x00, 0x64, 0x61, 0x74, 0x61, 0x40, 0xC5, 0x66, 0x1B, 0x87, 0xFF, 0xC3, 0xFF,
    0xA9, 0x00, 0xAF, 0x01, 0x2A, 0x03, 0x57, 0x04, 0x11, 0x05, 0x37, 0x05, 0x0A, 0x06, 0xD8, 0x06,
    0xBE, 0x07, 0x1E, 0x08, 0x90, 0x07, 0xB7, 0x05, 0x12, 0x04, 0x4D, 0x02, 0xB8, 0x01, 0x8A, 0x01,
    0x61, 0x00, 0xAF, 0xFF, 0x82, 0xFE, 0x58, 0xFE, 0x84, 0xFD, 0xD5, 0xFC, 0x83, 0xFD, 0xD2, 0xFE,
    0x82, 0xFF, 0xB9, 0xFF, 0xBD, 0xFF, 0xB9, 0x00, 0x4B, 0x01, 0x52, 0x01, 0xC6, 0x00, 0xA6, 0x00,
    0xB9, 0x00, 0x0B, 0x01, 0xE2, 0x01, 0xF5, 0x01, 0xE0, 0x02, 0x44, 0x04, 0x1C, 0x04, 0x80, 0x03,
    0x62, 0x04, 0x9E, 0x05, 0xA4, 0x06, 0xBC, 0x07, 0x14, 0x08, 0x33, 0x08, 0xFB, 0x08, 0x31, 0x09,
    0xF4, 0x09, 0xA6, 0x09, 0x3B, 0x08, 0x0B, 0x07, 0x96, 0x05, 0x2C, 0x04, 0xDE, 0x03, 0x0C, 0x04,
    0x22, 0x04, 0xF5, 0x04, 0xA4, 0x04, 0xF0, 0x03, 0x18, 0x03, 0x5E, 0x02, 0x92, 0x01, 0xBF, 0x01,
    0x7C, 0x01, 0x7E, 0x01, 0x83, 0x01, 0xA2, 0x02, 0x23, 0x03, 0xE7, 0x03, 0xB8, 0x04, 0x74, 0x03,
    0x17, 0x02, 0x75, 0x01, 0x92, 0x01, 0xAD, 0x01, 0x71, 0x01, 0x51, 0x00, 0xF8, 0xFF, 0x1F, 0xFF 
	};
	

/*
 * Server Index Page
 */
#if(1)
const char* serverIndex =
"<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.6.0/jquery.min.js'></script>"
"<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
"<input type='file' name='update'>"
"<input type='submit' value='Update'>"    
"</form>"
 "<progress id='progressbar' max='100' value='0'></progress>"   
 "<label id='lbl' style='display: none;'>Update finished!</label>"
 "<script>"
  "$('form').submit(function(e){"
  "e.preventDefault();"
  "var progressBar = $('#progressbar');"
  "var form = $('#upload_form')[0];"
  "var data = new FormData(form);"
  "$.ajax({"
  "url: '/update',"
  "type: 'POST',"
  "data: data,"
  "contentType: false,"
  "processData:false,"
  "xhr: function() {"
  "var xhr = new window.XMLHttpRequest();"
  "xhr.upload.addEventListener('progress', function(evt) {"
  "if (evt.lengthComputable) {"
  "var per = evt.loaded / evt.total;"
  "progressBar.val(Math.round(per*100)).text('Progress ' + Math.round(per*100) + '%');"
  "}"
  "}, false);"
  "return xhr;"
  "},"
  "success:function(d, s) {"
  "document.getElementById('lbl').style.display = 'inherit';"
  "console.log('success!')"
 "},"
 "error: function (a, b, c) {"
 "}"
 "});"
 "});"
 "</script>";

#else
 const char  *serverIndex = "<form method='POST' action='/update' enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='Update'></form>";
#endif

// const char* style =   "<style>div{padding:2px;font-size:1em;}body,textarea,input,select{background: 0;border-radius: 0;font: 16px sans-serif;margin: 0}textarea,input,select{outline: 0;font-size: 14px;border: 1px solid #ccc;padding: 8px;width: 90%}.btn a{text-decoration: none}.container{margin: auto;width: 90%}@media(min-width:1200px){.container{margin: auto;width: 30%}}@media(min-width:768px) and (max-width:1200px){.container{margin: auto;width: 50%}}.btn,h2{font-size: 2em}h1{font-size: 3em}.btn{background: #0ae;border-radius: 4px;border: 0;color: #fff;cursor: pointer;display: inline-block;margin: 2px 0;padding: 10px 14px 11px;width: 100%}.btn:hover{background: #09d}.btn:active,.btn:focus{background: #08b}label>*{display: inline}form>*{display: block;margin-bottom: 10px}textarea:focus,input:focus,select:focus{border-color: #5ab}.msg{background: #def;border-left: 5px solid #59d;padding: 1.5em}.q{float: right;width: 64px;text-align: right}.l{background: url('data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAACAAAAAgCAMAAABEpIrGAAAALVBMVEX///8EBwfBwsLw8PAzNjaCg4NTVVUjJiZDRUUUFxdiZGSho6OSk5Pg4eFydHTCjaf3AAAAZElEQVQ4je2NSw7AIAhEBamKn97/uMXEGBvozkWb9C2Zx4xzWykBhFAeYp9gkLyZE0zIMno9n4g19hmdY39scwqVkOXaxph0ZCXQcqxSpgQpONa59wkRDOL93eAXvimwlbPbwwVAegLS1HGfZAAAAABJRU5ErkJggg==') no-repeat left center;background-size: 1em}input[type='checkbox']{float: left;width: 20px}.table td{padding:.5em;text-align:left}.table tbody>:nth-child(2n-1){background:#ddd}fieldset{border-radius:0.5rem;margin:0px;}</style></head><body><div class='container'><div style='text-align:left;display:inline-block;min-width:260px;'>";
