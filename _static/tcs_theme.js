var i;
var contents = document.getElementsByClassName("content-collapse section");

for (i = 0; i < contents.length; i++) {

  //Make sure the "content-collapse section" class is occurring in <div>
  if (contents[i].tagName.toLowerCase() == 'div') {
    var element = contents[i].children[0];
    var element_type = element.tagName.toLowerCase();
    var span_id;
    var spanElement;

    //if the next element is a span grab the id and skip to the header
    if (element_type == 'span') {
      span_id = element.id;
      element.id = "";
      element = contents[i].children[1];
      element_type = element.tagName.toLowerCase();
    }

    var btn = document.createElement("BUTTON");
    //If it is a header capture which level and pass on to button
    if (element_type.length == 2 && element_type[0] == 'h') {
      var newClass = 'clps' + element_type[1];
      //collapses the section by default only if javascript is working
      contents[i].style.maxHeight = 0;
      //Build the button and define behavior
      btn.className += " " + newClass;
      btn.innerHTML = element.innerHTML;
      btn.className += " collapsible";
      btn.id = span_id;
      btn.addEventListener("click", function() {
        this.classList.toggle("active");
        var content = this.nextElementSibling;
        if (content.style.maxHeight != "0px"){
          content.style.maxHeight = 0;
        } else {
          content.style.maxHeight = content.scrollHeight + "px";
        } 
      });

      //Add the button to the page and remove the header
      contents[i].parentNode.insertBefore(btn, contents[i]);
      contents[i].removeChild(element);
    }else{
      //reset span id if it isn't followed by Hx element
      spanElement.id = span_id;
    }
  }
}