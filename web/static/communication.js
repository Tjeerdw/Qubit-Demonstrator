function orb_send_operation(string, to_disable = false)
{
  console.log('Sending command ' + string + '.')
  var req = new XMLHttpRequest()
  req.open('GET', '/operation/' + string)
  req.onreadystatechange = function () {
    if (this.readyState == 4 && this.status == 200) {
      set_button_status(false)
      window.num_turns++
      orb_status()
    }
  }
  set_button_status(true)
  if (to_disable !== false) {
    to_disable.used = true
  }
  req.send()
}

function orb_send_rotation(x,y,z,theta, to_disable = false)
{
  norm = Math.sqrt(x*x + y*y + z*z)
  if (Math.abs(norm - 1) > 1e-8)
  {
    alert("The norm of the goal vector has to be 1.")
  } else {
    console.log('Sending command u((' + x + ',' + y + ',' + z + '),' + theta + ').')
    var req = new XMLHttpRequest()
    req.open('GET', '/rotation/' + x + '/' + y + '/' + z + '/' + theta)
    req.onreadystatechange = function () {
      if (this.readyState == 4 && this.status == 200) {
        set_button_status(false)
        window.num_turns++
        orb_status()
      }
    }
    set_button_status(true)
    if (to_disable !== false) {
      to_disable.used = true
    }
    req.send()
  }
}

function set_button_status(status)
{
  buttons = document.getElementsByClassName('rot')
  for (i = 0; i < buttons.length; i++) {
    if (typeof buttons[i].used === 'undefined' || !buttons[i].used) {
      buttons[i].disabled = status
    }
  }
}

function orb_reset()
{
  console.log('Resetting orb.')
  window.num_turns = 0
  var req = new XMLHttpRequest()
  req.open('GET', '/reset')
  req.onreadystatechange = function () {
    if (this.readyState == 4 && this.status == 200) {
      console.log("Done resetting orb.")
    }
  }
  req.send()
}

function orb_set_goal(x,y,z)
{
  norm = Math.sqrt(x*x + y*y + z*z)
  if (Math.abs(norm - 1) > 1e-8)
  {
    alert("The norm of the goal vector has to be 1.")
  } else {
    console.log("Setting goal to [" + x.toString() + ", " + y.toString() + ", " + z.toString() + "].")
    var req = new XMLHttpRequest()
    req.open('GET', '/set_goal/' + x.toString() + '/' + y.toString() + '/' + z.toString())
    req.onreadystatechange = function () {
      if (this.readyState == 4 && this.status == 200) {
        console.log("Done setting goal.")
      }
    }
    req.send()
  }
}

function orb_status()
{
  console.log('Requesting status.')
  var req = new XMLHttpRequest()
  req.open('GET', '/status')
  req.onreadystatechange = function () {
    if (this.readyState == 4 && this.status == 200) {
      var res = this.responseText
      console.log("Status: " + res)
      if (res === "True") {
        document.getElementById('success').click()
      }
    }
  }
  req.send()
}

function remove_splash() {
  document.getElementById('wrapper').style.display = 'none';
}
